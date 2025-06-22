using Codice.Client.Common.TreeGrouper;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEngine;
using UnityEngine.Profiling;
using static OctNav.OctNavGraph;

namespace OctNav
{
    [RequireComponent(typeof(BoundsHandles))]
    public class OctVolume : MonoBehaviour
    {
        [Header("Settings")]
        public LayerMask geometryMask = ~0;
        public int maxDepth = 5;
        public float toleranceForFittingCubes = 0.5f;
        [Tooltip("If you're not using convex shapes there will be nodes AI will try to path to inside geometry, turning this on fixes this.")]
        public bool performCulling = false;
        [Header("Gizmos")]
        public bool drawGizmosOnlyWhenSelected = false;
        public bool showHitGizmos = false;
        public bool showOctreeGizmos = false;
        public bool showSurfaceObjectGizmos = false;
        public bool drawGraph = false;
        public bool drawGroundedGraph = false;
        public bool showEmptyLeavesGizmos = false;
        public bool selectionMode = false;
        public bool enableDistanceFade = false;
        public float fadeStartDistance = 100;
        public float fadeEndDistance = 200;

        [HideInInspector] public List<OctNode> allNodes = new List<OctNode>();
        [HideInInspector] public List<OctNode> hitNodes = new List<OctNode>();
        [HideInInspector] public HashSet<OctNode> walkableSet = new HashSet<OctNode>();
        [HideInInspector] public List<OctNode> emptyLeaves = new List<OctNode>();
        [HideInInspector] public BoundsHandles boundHandles;
        [HideInInspector] public Bounds bounds;
        [HideInInspector] public OctNode root;
        [HideInInspector] public int currentNodeIndex = 0;

        List<OctNode> culledNodes = new List<OctNode>();
        private Vector3 sceneCamPos;
        private List<Collider> surfaceObjects;
        private float biggestSide;
        private Bounds RootBounds;

        public static string OctreeSaveDirectory => Path.Combine(Directory.GetParent(Application.dataPath).FullName, "SceneData", "OctreeData");
        public string SavePath => Path.Combine(OctreeSaveDirectory, $"{gameObject.scene.name}_{gameObject.name}.json");

        private void Awake()
        {
            if (!LoadOctree())
            {
                Build();
            }
        }

        private void OnDestroy()
        {
            ResetOctree();
        }

        private void OnValidate()
        {
            if (boundHandles == null)
            { 
                boundHandles = GetComponent<BoundsHandles>(); 
            }

            boundHandles.OnBoundsChanged -= UpdateBounds;
            boundHandles.OnBoundsChanged += UpdateBounds;
            bounds = boundHandles.GetBounds();
        }

        private void UpdateBounds(Bounds newBounds)
        {
            bounds = newBounds;
        }

        /// <summary>
        /// Saves the current octree structure to disk.
        /// </summary>
        public void SaveOctree()
        {
            if (root == null) return;
            OctVolumeConvert.SaveFullVolume(SavePath, this);
        }

        /// <summary>
        /// Attempts to load a previously saved octree. Rebuilds graphs if successful.
        /// </summary>
        public bool LoadOctree()
        {
            ResetOctree();

            if (OctVolumeConvert.LoadFullVolume(SavePath, this))
            {
                BuildGraph();
                BuildWalkableGraph();
            }

#if UNITY_EDITOR
            SceneView.RepaintAll();
#endif
            return true;
        }

        /// <summary>
        /// Builds a new octree from geometry inside the current bounds.
        /// </summary>
        public void Build()
        {
            Profiler.BeginSample("Clean");
            float startTime = Time.realtimeSinceStartup;
            ResetOctree();

            biggestSide = Mathf.Max(bounds.size.x, bounds.size.y, bounds.size.z);
            Vector3 minCorner = bounds.min;
            Vector3 cubeSize = Vector3.one * biggestSide;
            Vector3 cubeCenter = (minCorner + cubeSize * 0.5f) + transform.position;
            RootBounds = new Bounds(cubeCenter, cubeSize);

            root = new OctNode(RootBounds.center, RootBounds.size, null);
            root.SubdivideRecursive(maxDepth, geometryMask, bounds);

            GetEmptyLeaves(root);

            BuildFaceLinks();
            int before = emptyLeaves.Count;
            CullUnreachableNodes();
            int culled = before - emptyLeaves.Count;

            BuildGraph();
            CollectAllNodes(root);
            BuildWalkableGraph();

            Debug.Log($"[OctNav] Volume build: {Time.realtimeSinceStartup - startTime:F6}s | Empty nodes: {emptyLeaves.Count} | Culled: {culled}");
#if UNITY_EDITOR
            SceneView.RepaintAll();
#endif
            SaveOctree();


            //finally
            //{
            //    foreach (var kvp in originalConvexStates)
            //        kvp.Key.convex = kvp.Value;
            //}
            Profiler.EndSample();
            Profiler.enabled = false;
        }

        /// <summary>
        /// Find all empty nodes in the volume and add them to the empty leaves list.
        /// </summary>
        public void GetEmptyLeaves(OctNode node)
        {
            if (node.isLeaf)
            {
                if (!node.hasCollision)
                {
                    emptyLeaves.Add(node);
                }
                return;
            }
            if (node.children == null) return;

            foreach (OctNode child in node.children)
            {
                GetEmptyLeaves(child);
            }
        }

        /// <summary>
        /// Adds empty leaf nodes and their connections to the main navigation graph.
        /// </summary>
        public void BuildGraph()
        {
            OctManager.graph.ClearVolume(this);
            foreach (OctNode leaf in emptyLeaves)
            {
                OctManager.graph.AddNode(this, leaf); 
            }

            foreach (OctNode leaf in emptyLeaves)
            {
                for (int i = 0; i < 6; i++)
                {
                    OctNode face = leaf.faceLinks[i];
                    if (face != null)
                    { 
                        OctManager.graph.AddEdge(leaf, face); 
                    }
                }
            }
        }

        /// <summary>
        /// Builds the grounded navigation graph for surface walking.
        /// </summary>
        public void BuildWalkableGraph()
        {
            OctManager.groundGraph.ClearVolume(this);

            List<OctNode> walkable = allNodes.Where(IsGroundTile).ToList();
            walkableSet = new HashSet<OctNode>(walkable);

            foreach (OctNode node in walkable)
            {
                for (int i = 0; i < 6; i++)
                { 
                    node.faceLinks[i] = node.GetFaceNeighbour((Direction)i); 
                }
            }

            foreach (OctNode node in walkable)
            {
                OctManager.groundGraph.AddNode(this, node, true);
                for (int dir = 0; dir < 6; dir++)
                {
                    OctNode neighbor = node.faceLinks[dir];
                    if (neighbor != null && walkableSet.Contains(neighbor))
                    { 
                        OctManager.groundGraph.AddEdge(node, neighbor);
                    }
                }
            }

            foreach (OctNode node in walkable)
            {
                List<Direction> missing = new List<Direction>();
                foreach (Direction dir in new[] { Direction.PosX, Direction.NegX, Direction.PosZ, Direction.NegZ })
                {
                    OctNode edgeNode = node.faceLinks[(int)dir];
                    if (edgeNode == null || !walkableSet.Contains(edgeNode))
                    {
                        missing.Add(dir);
                    }
                }

                if (missing.Count == 0) continue;

                node.edgeDirs.Clear();
                node.edgeDirs.AddRange(missing);
                node.isEdge = true;
/*
                foreach (Direction dir in missing)
                {
                    OctNode next = node.faceLinks[(int)dir];
                    if (next == null) continue;

                    OctNode up = next.GetFaceNeighbour(Direction.PosY);
                    while (up != null && up.hasCollision && up.isLeaf && !walkableSet.Contains(up))
                    { 
                        up = up.GetFaceNeighbour(Direction.PosY); 
                    }

                    if (up != null && walkableSet.Contains(up))
                    {
                        OctManager.groundGraph.AddEdge(node, up); 
                    }
                }*/
            }
        }
        private bool IsGroundTile(OctNode node)
        {
            // in future, set cull leaves to hit leaves
            OctNode top = node.GetFaceNeighbour(Direction.PosY);
            return node.hasCollision && node.isLeaf && top != null && !top.hasCollision;
        }

        private void CullUnreachableNodes()
        {
            Profiler.BeginSample("CullUnreachableNodes");

            // contains is so slow
            HashSet<OctNode> emptyLeafSet = new HashSet<OctNode>(emptyLeaves);

            OctNavGraph testGraph = new OctNavGraph();
            foreach (OctNode leaf in emptyLeaves)
            {
                testGraph.AddNode(this, leaf);
            }
            foreach (OctNode leaf in emptyLeaves)
            {
                for (int i = 0; i < 6; i++)
                {
                    OctNode neigh = leaf.faceLinks[i];
                    if (neigh != null && emptyLeafSet.Contains(neigh))
                    { 
                        testGraph.AddEdge(leaf, neigh); 
                    }
                }
            }

            List<OctNode> boundaryLeaves = new List<OctNode>(emptyLeaves.Count);
            foreach (OctNode leaf in emptyLeaves)
            {
                for (int i = 0; i < 6; i++)
                {
                    if (leaf.faceLinks[i] == null)
                    {
                        boundaryLeaves.Add(leaf);
                        break;
                    }
                }
            }

            HashSet<GraphNode> visited = new HashSet<GraphNode>();
            Queue<GraphNode> queue = new Queue<GraphNode>();
            foreach (OctNode leaf in boundaryLeaves)
            {
                GraphNode startNode = testGraph.FindNode(leaf);
                if (startNode != null)
                {
                    visited.Add(startNode);
                    queue.Enqueue(startNode);
                }
            }

            while (queue.Count > 0)
            {
                GraphNode current = queue.Dequeue();
                foreach (GraphEdge edge in current.edges)
                {
                    GraphNode neighbor = edge.a == current ? edge.b : edge.a;
                    if (!visited.Contains(neighbor))
                    {
                        visited.Add(neighbor);
                        queue.Enqueue(neighbor);
                    }
           
                }
            }
    /*        culledNodes = new List<OctNode>();  
            culledNodes = emptyLeaves.Where(l =>
            {
                GraphNode node = testGraph.FindNode(l);
                return node != null && !visited.Contains(node);
            }).ToList();*/

           emptyLeaves = emptyLeaves.Where(l =>
           {
               GraphNode node = testGraph.FindNode(l);
               if (node != null && visited.Contains(node))  return true;
               else if (node != null && !visited.Contains(node))
               {
                   l.hasCollision = true;
                   l.isLeaf = true;
               }
               return false;
           }).ToList();


            Profiler.EndSample();
            Profiler.enabled = false;
        }

        // MUST FIX IN FUTURE WAY SPEEDIER
 /*       private void CullUnreachableNodes()
        {
            HashSet<OctNode> visited = new HashSet<OctNode>();
            Queue<OctNode> queue = new Queue<OctNode>();

            foreach (OctNode leaf in emptyLeaves)
            {
                if (leaf.faceLinks.Any(n => n == null))
                {
                    visited.Add(leaf);
                    queue.Enqueue(leaf);
                }
            }

            while (queue.Count > 0)
            {
                OctNode current = queue.Dequeue();
                foreach (OctNode neighbor in current.faceLinks)
                {
                    if (neighbor != null && !visited.Contains(neighbor))
                    {
                        visited.Add(neighbor);
                        queue.Enqueue(neighbor);
                    }
                }
            }

            emptyLeaves = emptyLeaves.Where(visited.Contains).ToList();
        }*/

        private void BuildFaceLinks()
        {
            foreach (OctNode leaf in emptyLeaves)
            {
                for (Direction dir = Direction.PosX; dir <= Direction.NegZ; dir++)
                {
                    OctNode neighbor = leaf.GetFaceNeighbour(dir);
                    if (neighbor != null/* && !neighbor.isOutside*/)
                    {
                        leaf.faceLinks[(int)dir] = neighbor;
                    }
                }
            }
        }

    
        private void CollectAllNodes(OctNode node)
        {
            if (node == null) return;

            allNodes.Add(node);
            if (node.hasCollision)
            {
                hitNodes.Add(node);
            }

            if (node.children != null)
            {
                foreach (OctNode child in node.children)
                {
                    CollectAllNodes(child); 
                }
            }
        }

        public void ResetOctree()
        {
            OctManager.graph.ClearVolume(this);
            OctManager.groundGraph.ClearVolume(this);
            Collider[] hits = Physics.OverlapBox(bounds.center, bounds.size, Quaternion.identity, geometryMask);
            surfaceObjects = new List<Collider>(hits);
            root = new OctNode(RootBounds.center, RootBounds.size, null);
            allNodes.Clear();
            hitNodes.Clear();
            emptyLeaves.Clear();
            walkableSet.Clear();

#if UNITY_EDITOR
            SceneView.RepaintAll();
#endif
        }

        public void BuildSection(Bounds sectionBounds)
        {
            if (root == null)
            {
                Debug.LogWarning("Octree root is null");
                return;
            }

            List<OctNode> affected = new List<OctNode>(); 
            FindIntersectingLeaves(root, sectionBounds, affected);

            if (affected.Count == 0)
            {
                Debug.Log("No intersecting nodes found");
                return;
            }

            foreach (OctNode node in affected)
            {
                if (node.isLeaf)
                {
                    node.isLeaf = false;
                    node.hasCollision = false;
                    node.children = null;
                    node.SubdivideRecursive(maxDepth, geometryMask, bounds);
                }
            }

            emptyLeaves.Clear();
            GetEmptyLeaves(root);
            BuildFaceLinks();
            BuildGraph();
            BuildWalkableGraph();

#if UNITY_EDITOR
            SceneView.RepaintAll();
#endif
        }

        private void FindIntersectingLeaves(OctNode node, Bounds section, List<OctNode> result)
        {
            if (node == null || !node.bounds.Intersects(section)) return;

            if (node.isLeaf)
            {
                result.Add(node);
            }
            else if (node.children != null)
            {
                foreach (OctNode child in node.children)
                { 
                    FindIntersectingLeaves(child, section, result); 
                }
            }
        }

        public static int GetAllLeafNodesCount()
        {
            int count = 0;
            foreach (OctVolume vol in Resources.FindObjectsOfTypeAll<OctVolume>())
            {
                vol.emptyLeaves.Clear();
                vol.GetEmptyLeaves(vol.root);
                count += vol.emptyLeaves.Count;
            }
            return count;
        }

        public static List<OctNode> GetAllLeafNodesInScene()
        {
            List<OctNode> leaves = new List<OctNode>();
            foreach (OctVolume vol in Resources.FindObjectsOfTypeAll<OctVolume>())
            {
                vol.emptyLeaves.Clear();
                vol.GetEmptyLeaves(vol.root);
                leaves.AddRange(vol.emptyLeaves);
            }
            return leaves;
        }

        public OctNode FindNodeAtPoint(Vector3 point)
        {
            return root == null ? null : FindContainingNode(root, point);
        }

        private OctNode FindContainingNode(OctNode node, Vector3 point)
        {
            if (node.isLeaf) return node;
            foreach (var child in node.children ?? Array.Empty<OctNode>())
            {
                if (child.bounds.Contains(point))
                {
                    return FindContainingNode(child, point); 
                }
            }
            return null;
        }

        private void OnDrawGizmos()
        {
            if (!drawGizmosOnlyWhenSelected)
               {
                DrawGizmos(); 
            }
            foreach (OctNode nd in culledNodes)
            {

                Gizmos.color = OctColour.HotPink.Color();
                Gizmos.DrawWireCube(nd.bounds.center, nd.bounds.size * 0.8f);
            }
        }

        private void OnDrawGizmosSelected()
        {
            if (drawGizmosOnlyWhenSelected)
            {
                DrawGizmos();
            }
        }
        private void DrawGizmos()
        {
#if UNITY_EDITOR
            if (root != null && (showOctreeGizmos || showHitGizmos) && SceneView.currentDrawingSceneView)
            {
                Transform camTransform = SceneView.currentDrawingSceneView.camera.transform;
                sceneCamPos = camTransform.forward*2 + camTransform.position; 
                DrawNodeGizmo(root);
                DrawNodeGizmo(root, true);
            }
#endif
            if (showEmptyLeavesGizmos)
            {
                foreach (OctNode leaf in emptyLeaves)
                {
                    float dist = Vector3.Distance(leaf.bounds.center, sceneCamPos);
                    float alpha;
                    if (!enableDistanceFade)
                    {
                        alpha = 1f;
                    }
                    else if (dist <= fadeStartDistance)
                    {
                        alpha = 1f;
                    }
                    else if (dist >= fadeEndDistance)
                    {
                        alpha = 0f;
                    }
                    else
                    {
                        alpha = 1f - ((dist - fadeStartDistance) / (fadeEndDistance - fadeStartDistance));
                    }
                    Gizmos.color = Color.yellow * alpha;
                    Gizmos.DrawWireCube(leaf.bounds.center, leaf.bounds.size * 0.9f);
                }
            }
            if (drawGraph)
            {
                Gizmos.color = Color.cyan;
                OctManager.graph.DrawGraphGizmos();
            }
            if (drawGroundedGraph)
            {
                Gizmos.color = Color.yellow;
                OctManager.groundGraph.DrawGraphGizmos();
            }
            if (showSurfaceObjectGizmos)
            {
                foreach (Collider coll in surfaceObjects)
                {
                    Gizmos.color = Color.blue;
                    Gizmos.DrawWireCube(coll.bounds.center, coll.bounds.size);
                }
            }
            if (selectionMode && (allNodes != null && allNodes.Count != 0))
            {

                OctNode sel = allNodes[currentNodeIndex];
                if (sel != null)
                {
                    if (sel.faceLinks != null)
                    {
                        foreach (OctNode neighbour in sel.faceLinks)
                        {
                            if (neighbour != null)
                            {
                                if (neighbour.isLeaf) { 
                                    Gizmos.color = new Color(1f,0.6f,0,1);
                                }
                                else
                                {
                                    Gizmos.color = new Color(0, 0.6f, 1, 1);
                                }
                                Gizmos.DrawWireCube(neighbour.bounds.center, neighbour.bounds.size * 0.87f);
                            }
                        }
                    }
                    Gizmos.color = sel.isLeaf ?  new Color(1,0.6f,0,1) : new Color(0, 0.6f, 1, 1);
                    Gizmos.DrawWireCube(sel.bounds.center, sel.bounds.size * 0.88f);
                    OctNavGraph.GraphNode node = OctManager.graph.FindNode(sel);
                    if (node != null)
                    {
                        Gizmos.color = Color.cyan;
                        foreach (OctNavGraph.GraphEdge edge in node.edges)
                        {
                            OctNavGraph.GraphNode target = (edge.a == node) ? edge.b : edge.a;
                            Gizmos.DrawLine(node.center, target.center);
                            Gizmos.DrawSphere(target.center, 0.05f);
                        }
                    }
                }
            }

        }
        public void DrawNodeGizmo(OctNode node, bool hitPass = false)
        {
            if (node == null) return;

            float dist = Vector3.Distance(node.bounds.center, sceneCamPos);
            float alpha;
            if (!enableDistanceFade)
            {
                alpha = 1f;
            }
            else if (dist <= fadeStartDistance)
            {
                alpha = 1f;
            }
            else if (dist >= fadeEndDistance)
            {
                alpha = 0f;
            }
            else
            {
                alpha = 1f - ((dist - fadeStartDistance) / (fadeEndDistance - fadeStartDistance));
            }
            if (showOctreeGizmos && !hitPass && !node.isOutside && node.parent != null)
            {
                Gizmos.color = Color.green * alpha;
                Gizmos.DrawWireCube(node.bounds.center, node.bounds.size);
            }
         /*   if (showEmptyLeavesGizmos && !hitPass && !node.isOutside)
            {
                Gizmos.color = Color.green * alpha;
                Gizmos.DrawWireCube(node.bounds.center, node.bounds.size);
            }*/

            if (node.hasCollision && node.isLeaf && !node.isOutside && showHitGizmos && hitPass)
            {
                Gizmos.color = Color.red * alpha;
                Gizmos.DrawWireCube(node.bounds.center, node.bounds.size * 0.89f);
            }
         
            if (node.children != null)
            {
                foreach (OctNode child in node.children)
                {
                    DrawNodeGizmo(child, hitPass);
                }
            }
          
        }

    }


}
