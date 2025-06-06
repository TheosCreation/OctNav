using System;
using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEngine;
using UnityEngine.Profiling;


namespace OctNav {
    public enum Direction { PosX = 0, NegX = 1, PosY = 2, NegY = 3, PosZ = 4, NegZ = 5 }

    public class OctNode
    {
        public Bounds bounds;
        public OctNode[] children = new OctNode[0];
        public int depth = 0;
        public bool isLeaf = false;
        public bool hasCollision = false;
        public OctNode[] faceLinks = new OctNode[6];
        public OctNode[] neighbourLinks;
        OctNode parent;
        Collider[] overlapBufer = new Collider[1];  //unused
        public OctNode(Vector3 center, Vector3 size, OctNode parent)
        {
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
                children[i] = new OctNode(center, childSize, this);
            }
        }

        public void SubdivideRecursive(int maxDepth, LayerMask geometryMask)
        {

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
                child.SubdivideRecursive(maxDepth, geometryMask);
            }

        }

    }

    [RequireComponent(typeof(BoundsHandles))]
    public class OctVolume : MonoBehaviour
    {
        [HideInInspector] public List<OctNode> allNodes = new List<OctNode>();
        public int currentNodeIndex = 0;
        [HideInInspector] public BoundsHandles boundHandles;
        [HideInInspector] public Bounds bounds;
        OctNode root;
        public LayerMask geometryMask = ~0;
        public int maxDepth = 5;
        public float toleranceForFittingCubes = 0.5f; //50%
        [Header("Big cube or LongCube")]
        public bool BigCube = true;

        [Header("Pathfinding settings")]
        public bool calculateFaces = false;

        [Header("Gizmos")]
        public bool drawGizmosOnlyWhenSelected = false;
        public bool showHitGizmos = false;
        public bool showOctreeGizmos = false;
        public bool showSurfaceObjectGizmos = false;
        public bool drawAStarGizmos = false;
        public bool selectionMode = false;
        public bool enableDistanceFade = false;
        public float fadeStartDistance = 100;
        public float fadeEndDistance = 200;

        Vector3 sceneCamPos;
        List<Collider> surfaceObjects;
        float biggestSide;
        float smallestLeaf;




        public List<OctNode> emptyLeaves = new List<OctNode>();
        public static string OctreeSaveDirectory => Path.Combine(Directory.GetParent(Application.dataPath).FullName, "SceneData", "OctreeData");
        public string SavePath => Path.Combine(OctreeSaveDirectory, $"{gameObject.scene.name}_{gameObject.name}.json");

        private void Awake()
        {
            if(LoadOctree())
            {
                BuildChildren();
            }
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

        public bool LoadOctree()
        {
            root = OctVolumeConvert.LoadFromFile(SavePath);
            if (root == null)
                return false;

            emptyLeaves.Clear();
            OctNavigation.graph = new Graph();
            GetEmptyLeaves(root);
            CollectAllNodes(root);

            if (calculateFaces)
            {
                BuildFaceLinks();
                BuildGraphFaces();
            }
       

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
                foreach (var child in node.children)
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

        public void GetEmptyLeaves(OctNode node)
        {
            if (node.isLeaf)
            {
                if (!node.hasCollision)
                {

                    if (bounds.Contains(node.bounds.min) && bounds.Contains(node.bounds.max))
                    {
                        emptyLeaves.Add(node);
                    }
                    //graph.AddNode(node);
                }
                return;
            }
            if (node.children == null) return;

            foreach (OctNode child in node.children)
            {
                GetEmptyLeaves(child);
            }
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
        void BuildGraphFaces()
        {
            OctNavigation.graph = new Graph();
            foreach (OctNode leaf in emptyLeaves)
            {
                OctNavigation.graph.AddNode(leaf); 
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
       

        public void BuildChildren()
        {
     /*       Collider[] hits = Physics.OverlapBox(bounds.center, bounds.size, Quaternion.identity, geometryMask);
            surfaceObjects = new List<Collider>(hits);*/
            Profiler.BeginSample("OctVolume-Build");
            float startTime = Time.realtimeSinceStartup;

            emptyLeaves.Clear();
            OctNavigation.graph = new Graph();

            if (BigCube) {
                //                     BIG CUBE method start
            Bounds RootBounds = bounds;
            biggestSide = Mathf.Max(bounds.size.x, bounds.size.y, bounds.size.z);
            Vector3 minCorner = bounds.min;
            Vector3 cubeSize = Vector3.one * biggestSide;
            Vector3 cubeCenter = minCorner + cubeSize * 0.5f; ;
            RootBounds = new Bounds(cubeCenter, cubeSize);

           /* smallestLeaf = biggestSide
                * Mathf.Pow(0.5f, maxDepth - 1);*/
            root = new OctNode(RootBounds.center, RootBounds.size, null);
            root.SubdivideRecursive(maxDepth, geometryMask);
            }
            //                 BIG CUBE method end    */

            float elapsed = Time.realtimeSinceStartup - startTime;
            Profiler.EndSample();
            Debug.Log($"Main thread took {elapsed} to compute physics using bounds with a depth of {maxDepth}");

            float se1 = Time.realtimeSinceStartup;
            GetEmptyLeaves(root);

            if (emptyLeaves.Count == 0)
            {
                Debug.LogWarning("No empty leaves found");
            }

            if (calculateFaces)
            {
                BuildFaceLinks();
                BuildGraphFaces();
            }
       


            float e1 = Time.realtimeSinceStartup - se1;        

            Debug.Log($"Leaves took {e1} to compute");

            Profiler.enabled = false;
        
#if UNITY_EDITOR
            UnityEditor.SceneView.RepaintAll();
#endif
            CollectAllNodes(root);
            SaveOctree();
        }

        bool CheckCollisionData(Bounds nodeBounds, Bounds[] colliderBounds)
        {
            foreach (var b in colliderBounds)
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
                foreach (var c in node.children)
                    CollectAllNodes(c);
            }
        }


        public void Reset()
        {
            Collider[] hits = Physics.OverlapBox(bounds.center, bounds.size, Quaternion.identity, geometryMask);
            surfaceObjects = new List<Collider>(hits);
            root = new OctNode(bounds.center, bounds.size, null);
            OctNavigation.graph = new Graph();
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

        private void OnDrawGizmos()
        {
            if (!drawGizmosOnlyWhenSelected)
            {
                DrawGizmos();
            }
        }
        private void OnDrawGizmosSelected()
        {
            
            if(drawGizmosOnlyWhenSelected)
            {
                DrawGizmos();
            }
        }

        private void DrawGizmos()
        {
           
            if (root != null && (showOctreeGizmos || showHitGizmos) && SceneView.currentDrawingSceneView)
            {
                sceneCamPos = SceneView.currentDrawingSceneView.camera.transform.position;
                DrawNodeGizmo(root);
                DrawNodeGizmo(root,true);
            }
            
            if (drawAStarGizmos)
            {
                OctNavigation.graph.DrawGraph();
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
                   
/*
                    if (sel.children != null)
                    {
                        Gizmos.color = Color.yellow;
                        foreach (OctNode child in sel.children)
                        {
                            Gizmos.DrawWireCube(child.bounds.center, child.bounds.size);
                        }
                    }*/
                    if (sel.faceLinks != null)
                    {
                        Gizmos.color = Color.magenta;
                        foreach (OctNode neighbour in sel.faceLinks)
                        {
                            if (neighbour != null)
                            {
                                Gizmos.DrawWireCube(neighbour.bounds.center, neighbour.bounds.size);
                            }
                        }
                    }
                    Gizmos.color = Color.blue;
                    Gizmos.DrawWireCube(sel.bounds.center, sel.bounds.size);
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
            if (showOctreeGizmos && !hitPass)
            { 
                Gizmos.color = Color.green*alpha;
                Gizmos.DrawWireCube(node.bounds.center, node.bounds.size);
            }
            if (node.hasCollision && node.isLeaf && showHitGizmos && hitPass)
            {
                Gizmos.color = Color.red* alpha;
                Gizmos.DrawWireCube(node.bounds.center, node.bounds.size);
            }

            if (node.children != null)
            {
                foreach (OctNode child in node.children)
                {
                    DrawNodeGizmo(child, hitPass); 
                }
            }
       

        }

        // Runtime function for user to rebuild little sections of the volume
        public void BuildSection(Bounds sectionBounds)
        {
            if (root == null)
            {
                Debug.LogWarning("Octree root is null. Cannot build section.");
                return;
            }

            List<OctNode> affectedNodes = new List<OctNode>();
            FindIntersectingLeaves(root, sectionBounds, affectedNodes);

            if (affectedNodes.Count == 0)
            {
                Debug.Log("No intersecting nodes found.");
                return;
            }

            foreach (OctNode node in affectedNodes)
            {
                if (node.isLeaf)
                {
                    node.isLeaf = false;
                    node.hasCollision = false;
                    node.children = null; 
                    node.SubdivideRecursive(maxDepth, geometryMask);
                }
            }

            emptyLeaves.Clear();
            OctNavigation.graph = new Graph();
            GetEmptyLeaves(root);

            if (calculateFaces)
            {
                BuildFaceLinks();
                BuildGraphFaces();
            }
         

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
    }

  
}
