using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEngine;
using UnityEngine.Profiling;


namespace OctNav
{
    public enum Direction { PosX = 0, NegX = 1, PosY = 2, NegY = 3, PosZ = 4, NegZ = 5 }

    public class OctNode
    {
        private static int nextId;
        public readonly int id;

        public Bounds bounds;
        public OctNode[] children = new OctNode[0];
        public int depth = 0;
        private const int MaxPossibleDepth = 16;
        public bool isOutside = false;
        public bool isLeaf = false;
        public bool isEdge = false;
        public bool hasCollision = false;
        public List<Direction> edgeDirs = new List<Direction>();
        public OctNode[] faceLinks = new OctNode[6];
        public OctNode[] neighbourLinks;
        public OctNode parent;
        public int SiblingIndex { get; private set; }
        Collider[] overlapBufer = new Collider[1];  //unused

        private static readonly int[] _scratchPath = new int[MaxPossibleDepth];
        private static int _scratchPathLen;

        public OctNode(Vector3 center, Vector3 size, OctNode parent)
        {
            id = nextId++;
            children = new OctNode[0];
            bounds = new Bounds(center, size);
            this.parent = parent;
            depth = parent!= null ? parent.depth + 1 : 0;
            children = null;
        }
        public bool CheckCollision(LayerMask geometryMask)
        {
            return Physics.OverlapBoxNonAlloc(bounds.center, bounds.extents, overlapBufer, Quaternion.identity, geometryMask, QueryTriggerInteraction.Ignore) > 0;
        }
        
        public OctNode GetFaceNeighbour(Direction dir)
        {
            int axis = (int)dir / 2;
            int sign = ((int)dir % 2 == 0) ? +1 : -1;
            int targetBit = (sign > 0) ? 0 : 1;


            ///     <-- Testing some optimisation -->
            ///     
            /// Replaces per-call List allocations with a single preallocated int[] scratch buffer, eliminating heap churn.
            /// Stores each node's slot as SiblingIndex to avoid expensive Array.IndexOf scans, making child lookups O(1).
            /// Uses bit-flips on packed child-index bits in FaceNeighbourAtDepth for constant-time neighbour traversal without allocations.
            /// 
            /// Flip if to check change REMOVE before release


            if (false)
            {
               /* // walk up, record each slot index
                _scratchPathLen = 0;
                OctNode walk = this;
                while (walk.parent != null)
                {
                    _scratchPath[_scratchPathLen++] = walk.SiblingIndex;
                    walk = walk.parent;
                }

                // walk back down through the neighbour faces
                OctNode node = walk;  // now at the root
                for (int i = _scratchPathLen - 1; i >= 0; i--)
                {
                    node = node.FaceNeighbourAtDepth(_scratchPath[i], dir);
                    if (node == null) break;
                }

                return node;*/
            }
            else
            {
                List<int> path = new List<int>();
                OctNode walk = this;
                while (walk.parent != null)
                {
                    int index = Array.IndexOf(walk.parent.children, walk);
                    path.Add(index);
                    walk = walk.parent;
                }



                /*
                    store the index of each node going up to the root so we can find 
                    this nodes index relative to each depth level, path[0] is our index at our parents

                 */





                int branchLevel = -1;
                OctNode branchNode = this;
                for (int level = 0; level < path.Count; level++)
                {
                    int index = path[level];
                    int bit = (index >> axis) & 1;
                    /* 
                      this nodes index matches its axis so shifting it by the directions axis 
                      so 000 (0) is -x -y -z and 111 is +x +y +z  (7)
                      so shifting this by axis which will move it so that the first index (0,1) is the axis
                      & 1 turns it down to 1 or 0, as possible or false, then we can check if its the reverse 
                      of the sign to check if there is a valid sibling in that direction 

                      so we go through each level and break out at the first level where 
                      the direction is going towards a valid sibling 
                      then it sets branch level to that common ancestor 
                    */
                    if ((sign > 0 && bit == 0) || (sign < 0 && bit == 1))
                    {
                        branchLevel = level;
                        break;
                    }
                    branchNode = branchNode.parent;
                }
                if (branchLevel < 0)
                {
                    return null; // no neighbour in that direction
                }

                int siblingIndex = path[branchLevel] ^ (1 << axis);
                OctNode neighbour = branchNode.parent.children[siblingIndex];
                /* 
                   1 gets shifed left by axis amount so theres only one 1 at the axis pos 
                                    (x = 001, y = 010, z = 100) 
                   then xoring it with the index will give the sibling in that axis's index 
                */


                for (int level = branchLevel - 1; level >= 0; level--)
                {
                    //if (neighbour.isLeaf)
                    if (neighbour.children == null || neighbour.children.Length == 0)
                    {
                        break;
                    }
                    /*
                        then decend down the path untill you get to this nodes level 
                        if you reach a leaf it means that is the neighbour in that direction
                    */

                    int childIndex = 0;
                    for (int a = 0; a < 3; a++)
                    {
                        int b = (a == axis) ? targetBit : ((path[level] >> a) & 1);
                        /*
                            then to get the index in the correct position from that neighbour
                            loop through each axis, check if its on the correct axis from
                            the original direction, if it is then set set b to the reverse of the direction 
                            otherwise set it to the axis of the index at that level
                        */
                        childIndex |= (b << a);
                        /*
                           then we just add it to the child index, so first it adds the 1 bit then 2 then 4
                        */
                    }
                    neighbour = neighbour.children[childIndex];
                    /*
                        finally repeat untill either you reach a leaf
                    */
                }

                return neighbour;
            }
        }

        public OctNode FaceNeighbourAtDepth(int childIndex, Direction dir)
        {
            // If we haven’t subdivided, no deeper neighbour exists
            if (children == null)
                return null;

            // Decode the bits of childIndex (x=bit0, y=bit1, z=bit2)
            int bitX = (childIndex & 1);
            int bitY = ((childIndex >> 1) & 1);
            int bitZ = ((childIndex >> 2) & 1);

            // Determine axis and check boundary condition:
            // e.g. to go +X the original bitX must be 1 (you’re on the +X face)
            switch (dir)
            {
                case Direction.PosX:
                    if (bitX == 0) return null;
                    bitX = 0;    // flip from 1 to 0 to step into neighbour’s left-side octant
                    break;

                case Direction.NegX:
                    if (bitX == 1) return null;
                    bitX = 1;    // flip from 0 to 1
                    break;

                case Direction.PosY:
                    if (bitY == 0) return null;
                    bitY = 0;
                    break;

                case Direction.NegY:
                    if (bitY == 1) return null;
                    bitY = 1;
                    break;

                case Direction.PosZ:
                    if (bitZ == 0) return null;
                    bitZ = 0;
                    break;

                case Direction.NegZ:
                    if (bitZ == 1) return null;
                    bitZ = 1;
                    break;

                default:
                    throw new ArgumentOutOfRangeException(nameof(dir), dir, null);
            }

            // Recombine bits into a child index (z<<2 | y<<1 | x)
            int neighbourChildIndex = (bitZ << 2) | (bitY << 1) | bitX;

            // Return that child (may be null if it wasn’t split at all)
            return children[neighbourChildIndex];
        }

        public void Split()
        {
            Vector3 childSize = bounds.size * 0.5f;
            Vector3 childQuarter = bounds.size * 0.25f;
            children = new OctNode[8];

            for (int i = 0; i < 8; i++)
            {
                // Extract axis bits using bit shifts for the Z-Order Curve
                int x = (i >> 0) & 1;
                int y = (i >> 1) & 1;
                int z = (i >> 2) & 1;

                int sx = (x == 0) ? -1 : 1;
                int sy = (y == 0) ? -1 : 1;
                int sz = (z == 0) ? -1 : 1;

                Vector3 offset = new Vector3(
                    sx * childQuarter.x,
                    sy * childQuarter.y,
                    sz * childQuarter.z
                );

                Vector3 center = bounds.center + offset;
                var child = new OctNode(center, childSize, this);
                child.SiblingIndex = i;
                children[i] = child;
            }
        }

        public void SubdivideRecursive(int maxDepth, LayerMask geometryMask, Bounds volumeBounds)
        {

            if (!volumeBounds.Intersects(bounds))
            {
                isLeaf = true;
                hasCollision = true;
                isOutside = true;
                return;
            }

            hasCollision = CheckCollision(geometryMask);
            if(depth>=maxDepth)
            {
                isLeaf = true;
                return;
            }
            if (!hasCollision)
            {
                isLeaf = true;
                return;
            }
            Split();

            foreach (OctNode child in children)
            {
                child.SubdivideRecursive(maxDepth, geometryMask, volumeBounds);
            }

        }

    }

    [RequireComponent(typeof(BoundsHandles))]
    public class OctVolume : MonoBehaviour
    {
        [HideInInspector] public List<OctNode> allNodes = new List<OctNode>();
        [HideInInspector] public List<OctNode> hitNodes = new List<OctNode>();
        [HideInInspector] public HashSet<OctNode> walkableSet = new HashSet<OctNode>();
        [HideInInspector] public BoundsHandles boundHandles;
        [HideInInspector] public Bounds bounds;
        [HideInInspector] public OctNode root;
        public LayerMask geometryMask = ~0;
        public int maxDepth = 5;
        public float toleranceForFittingCubes = 0.5f; //50%
        public bool setCollidersToConex = false;

        //[Header("Pathfinding settings")]

        [Header("Gizmos")]
        public bool drawGizmosOnlyWhenSelected = false;
        public bool showHitGizmos = false;
        public bool showOctreeGizmos = false;
        public bool showSurfaceObjectGizmos = false;
        public bool drawGraph = false;
        public bool drawGroundedGraph = false;
        public bool showEmptyLeavesGizmos = false;
        public bool selectionMode = false;
        [HideInInspector]  public int currentNodeIndex = 0;
        public bool enableDistanceFade = false;
        public float fadeStartDistance = 100;
        public float fadeEndDistance = 200;

        Vector3 sceneCamPos;
        List<Collider> surfaceObjects;
        float biggestSide;
        Bounds RootBounds;



        public List<OctNode> emptyLeaves = new List<OctNode>();

        public static string OctreeSaveDirectory => Path.Combine(Directory.GetParent(Application.dataPath).FullName, "SceneData", "OctreeData");
        public string SavePath => Path.Combine(OctreeSaveDirectory, $"{gameObject.scene.name}_{gameObject.name}.json");

        private void Awake()
        {
            if(!LoadOctree())
            {
                BuildChildren();
            }
           // DontDestroyOnLoad(this.gameObject);
        }

        /*
         optimize using this in the future
        https://graphics.cs.kuleuven.be/publications/BLD13OCCSVO/
        https://github.com/gabrielmuller/zazen/blob/master/src/construct/construct.cpp
         */
        public void SaveOctree()
        {
            if (root == null)
            {
                Debug.LogWarning("No octree to save.");
                return;
            }

            OctVolumeConvert.SaveToFile(SavePath, root);
        }

        private void OnDestroy()
        {
            ResetOctree();
        }

        public bool LoadOctree()
        {
            ResetOctree();
            root = OctVolumeConvert.LoadFromFile(SavePath);
            if (root == null) return false;

            //emptyLeaves.Clear();

            //GetEmptyLeaves(root);
            //CollectAllNodes(root);

          //  BuildFaceLinks();
            //CullUnreachableNodes();
           // BuildGraph();
           // BuildWalkableGraph();
       

            #if UNITY_EDITOR
                SceneView.RepaintAll();
            #endif

            return true;
        }

        SerializableOctNode SerializeNode(OctNode node)
        {
            SerializableOctNode data = new SerializableOctNode
            {
                bounds = new SerializableBounds(node.bounds),
                isLeaf = node.isLeaf,
                hasCollision = node.hasCollision,
                depth = node.depth
            };

            if (node.children != null)
            {
                foreach (OctNode child in node.children)
                {
                    data.children.Add(SerializeNode(child));
                }
            }

            return data;
        }

        OctNode DeserializeNode(SerializableOctNode data, OctNode parent)
        {
            OctNode node = new OctNode(data.bounds.center.ToVector3(), data.bounds.size.ToVector3(), parent)
            {
                isLeaf = data.isLeaf,
                hasCollision = data.hasCollision,
                depth = data.depth
            };

            if (data.children.Count > 0)
            {
                node.children = new OctNode[8];
                for (int i = 0; i < data.children.Count; i++)
                {
                    node.children[i] = DeserializeNode(data.children[i], node);
                }
            }

            return node;
        }

        public void BuildChildren()
        {
            Profiler.BeginSample("OctVolume-Build");
            float startTime = Time.realtimeSinceStartup;
            ResetOctree();

            // workaround to non-convex mesh colliders, looks extremely illegal
            Collider[] hits = Physics.OverlapBox(bounds.center, bounds.size, Quaternion.identity, geometryMask);
            surfaceObjects = new List<Collider>(hits);
            Dictionary<MeshCollider, bool> originalConvexStates = new Dictionary<MeshCollider, bool>();
            
            try {
                if (setCollidersToConex) { 
                    foreach (Collider coll in hits)
                    {
                        MeshCollider meshCollider = coll as MeshCollider;
                        if (meshCollider != null && !meshCollider.convex)
                        {
                            originalConvexStates[meshCollider] = meshCollider.convex;
                            meshCollider.convex = true;
                        }
                    }
                }
                biggestSide = Mathf.Max(bounds.size.x, bounds.size.y, bounds.size.z);

                Vector3 minCorner = bounds.min;
                Vector3 cubeSize = Vector3.one * biggestSide;
                Vector3 cubeCenter = (minCorner + cubeSize * 0.5f) + transform.position;
                RootBounds = new Bounds(cubeCenter, cubeSize);

                root = new OctNode(RootBounds.center, RootBounds.size, null);
                root.SubdivideRecursive(maxDepth, geometryMask, bounds);

                float elapsed = Time.realtimeSinceStartup - startTime;
                Profiler.EndSample();
                Debug.Log($"Main thread took {elapsed} to compute physics using bounds with a depth of {maxDepth}");

                float se1 = Time.realtimeSinceStartup;
                GetEmptyLeaves(root);

                if (emptyLeaves.Count == 0)
                {
                    Debug.LogWarning("No empty leaves found");
                }

                BuildFaceLinks();

                CullUnreachableNodes();

                BuildGraph();

                CollectAllNodes(root);

                BuildWalkableGraph();
            

                float e1 = Time.realtimeSinceStartup - se1;

                Debug.Log($"Leaves took {e1} to compute");

                Profiler.enabled = false;

#if UNITY_EDITOR
                UnityEditor.SceneView.RepaintAll();
#endif

                SaveOctree();
            }
            finally
            {
                foreach (var kvp in originalConvexStates)
                {
                    kvp.Key.convex = kvp.Value;
                }
            }



        }

       public void GetEmptyLeaves(OctNode node) 
        {
            if (node.isLeaf )
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
        public void BuildGraph()
        {
            OctNavigation.graph.ClearVolume(this);
            foreach (OctNode leaf in emptyLeaves)
            {
                OctNavigation.graph.AddNode(this, leaf); 
            }
            foreach (OctNode leaf in emptyLeaves)
            {
                for (int i = 0; i < 6; i++)
                {
                    OctNode faceNode = leaf.faceLinks[i];
                    if (faceNode != null)
                    {
                        OctNavigation.graph.AddEdge(leaf, faceNode); 
                    }
                }
            }
        }
        public void CullUnreachableNodes()
        {
            BuildFaceLinks();

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
                for (int dir = 0; dir < 6; dir++)
                {
                    OctNode neighbor = current.faceLinks[dir];
                    if (neighbor != null && !visited.Contains(neighbor) && !neighbor.hasCollision)
                    {
                        visited.Add(neighbor);
                        queue.Enqueue(neighbor);
                    }
                }
            }

            Debug.Log(emptyLeaves.Count + " empty leaves before culling, " + visited.Count + " after culling.");
            emptyLeaves = emptyLeaves
                .Where(leaf => visited.Contains(leaf))
                .ToList();
        }

        void BuildFaceLinks()
        {
            foreach (OctNode leaf in emptyLeaves)
            {
                for (Direction dir = Direction.PosX; dir <= Direction.NegZ; dir++)
                {
                    leaf.faceLinks[(int)dir] = leaf.GetFaceNeighbour(dir);
                }
            }
        }

        public void BuildWalkableGraph()
        {
            OctNavigation.groundGraph.ClearVolume(this); // remove all old nodes that may have been left from previous build

            List<OctNode> walkable = allNodes.Where(n => IsGroundTile(n)).ToList();
            // finds hit tiles that dont have any hit tiles aboce it
            walkableSet = new HashSet<OctNode>(walkable);

            foreach (OctNode node in walkable) { 
                for (int i = 0; i < 6; i++) {
                    node.faceLinks[i] = node.GetFaceNeighbour((Direction)i);
                }
            }
            foreach (OctNode node in walkable)
            {
                OctNavigation.groundGraph.AddNode(this, node,true);
                for (int dir = 0; dir < 6; dir++)
                {
                    OctNode neighbor = node.faceLinks[dir];
                    if (neighbor != null && walkableSet.Contains(neighbor))
                    {
                        OctNavigation.groundGraph.AddEdge(node, neighbor);
                     
                    }
                }
            }
            foreach (OctNode node in walkable)
            {
                List<Direction> missing = new List<Direction>();
                foreach (int dir in new[] { (int)Direction.PosX, (int)Direction.NegX, (int)Direction.PosZ, (int)Direction.NegZ })
                {
                    OctNode edgeNode = node.faceLinks[dir];
                    if (edgeNode == null || !walkableSet.Contains(edgeNode))
                    {
                        missing.Add((Direction)dir); 
                    }
                }
                if (missing.Count == 0) continue;
                node.edgeDirs.Clear();
                node.edgeDirs.AddRange(missing);
                node.isEdge = true;

                foreach (Direction dir in missing)
                {
                    OctNode nextNode = node.faceLinks[(int)dir];
                    if(nextNode == null) continue;

                    OctNode up = nextNode.GetFaceNeighbour(Direction.PosY);
                    
                    while (up != null
                       && up.hasCollision
                       && up.isLeaf
                       && !walkableSet.Contains(up))
                    {
                       // DrawBounds2D(up.bounds, Color.magenta);

                        up = up.GetFaceNeighbour(Direction.PosY); //faceLinks[(int)Direction.PosY];
                    }
                    if (up != null && walkableSet.Contains(up))
                    {
                        OctNavigation.groundGraph.AddEdge(node, up);
                    }


                    /*OctNode bot = nextNode.GetFaceNeighbour(Direction.NegY);

                    while (bot != null
                       && bot.hasCollision
                       && bot.isLeaf
                       && !walkableSet.Contains(bot))
                    {
                        // DrawBounds2D(up.bounds, Color.magenta);

                        bot = bot.GetFaceNeighbour(Direction.NegY); //faceLinks[(int)Direction.PosY];
                    }
                    if (bot != null && walkableSet.Contains(bot))
                    {
                        groundGraph.AddEdge(node, bot);
                    }*/


                }
            }
        }
        
        bool IsGroundTile(OctNode node)
        {
            OctNode topNode = node.GetFaceNeighbour(Direction.PosY);
            if (node.hasCollision && node.isLeaf && topNode != null)
            {
                if (!topNode.hasCollision)
                {
                    return true;
                }
            }
            return false;
        }
     
        bool CheckCollisionData(Bounds nodeBounds, Bounds[] colliderBounds)
        {
            foreach (Bounds b in colliderBounds)
            {
                if (b.Intersects(nodeBounds))
                {
                    return true;
                }
            }
            return false;
        }

        void CollectAllNodes(OctNode node)
        {
            if (node == null) return;
            allNodes.Add(node);
            if (node.children != null)
            {
                if (node.hasCollision) 
                {
                    hitNodes.Add(node);
                }
                foreach (OctNode child in node.children)
                {
                    CollectAllNodes(child); 
                }
            }
        }

        public void ResetOctree()
        {
            Collider[] hits = Physics.OverlapBox(bounds.center, bounds.size, Quaternion.identity, geometryMask);
            surfaceObjects = new List<Collider>(hits);
            root = new OctNode(RootBounds.center, RootBounds.size, null);
            allNodes.Clear();
            hitNodes.Clear();
            emptyLeaves.Clear();
            OctNavigation.graph.ClearVolume(this);
            OctNavigation.groundGraph.ClearVolume(this);
            walkableSet.Clear();

#if UNITY_EDITOR
            UnityEditor.SceneView.RepaintAll();
#endif
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
        public void BuildSection(Bounds sectionBounds)
        {
            if (root == null)
            {
                Debug.LogWarning("octree root is null");
                return;
            }

            List<OctNode> affectedNodes = new List<OctNode>();
            FindIntersectingLeaves(root, sectionBounds, affectedNodes);

            if (affectedNodes.Count == 0)
            {
                Debug.Log("no intersecting nodes found");
                return;
            }

            foreach (OctNode node in affectedNodes)
            {
                if (node.isLeaf)
                {
                    node.isLeaf = false;
                    node.hasCollision = false;
                    node.children = null; 
                    node.SubdivideRecursive(maxDepth, geometryMask,bounds);
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
            if (node == null) return;

            if (!node.bounds.Intersects(section)) return;

            if (node.isLeaf)
            {
                result.Add(node);
                return;
            }

            if (node.children != null)
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
            OctVolume[] volumes = Resources.FindObjectsOfTypeAll<OctVolume>();
            foreach (OctVolume vol in volumes)
            {
                vol.emptyLeaves.Clear();
                vol.GetEmptyLeaves(vol.root);
                count += vol.emptyLeaves.Count;
            }
            return count;
        }
        public static List<OctNode> GetAllLeafNodesInScene()
        {
            List<OctNode> allLeaves = new List<OctNode>();
            OctVolume[] volumes = Resources.FindObjectsOfTypeAll<OctVolume>();
            foreach (OctVolume vol in volumes)
            {
                vol.emptyLeaves.Clear();
                vol.GetEmptyLeaves(vol.root);
                allLeaves.AddRange(vol.emptyLeaves);
            }
            return allLeaves;
        }
        public OctNode FindNodeAtPoint(Vector3 point)
        {
            if (root == null)
            {
                Debug.LogWarning("No octree set");
                return null;
            }
            return FindContainingNode(root, point);
        }
        private OctNode FindContainingNode(OctNode node, Vector3 point)
        {
            if (node.isLeaf) return node;
            foreach (OctNode child in node.children ?? Array.Empty<OctNode>())
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
                OctNavigation.graph.DrawGraph();
            }
            if (drawGroundedGraph)
            {
                Gizmos.color = Color.yellow;
                OctNavigation.groundGraph.DrawGraph();
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
                    Node node = OctNavigation.graph.FindNode(sel);
                    if (node != null)
                    {
                        Gizmos.color = Color.cyan;
                        foreach (Edge edge in node.edges)
                        {
                            Node target = (edge.a == node) ? edge.b : edge.a;
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
