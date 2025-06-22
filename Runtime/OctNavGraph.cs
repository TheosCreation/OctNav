using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace OctNav
{
    /// <summary>
    /// Defines the type of heuristic function to use in the A* pathfinding algorithm.
    /// </summary>
    public enum HeuristicType
    {
        Morrisium,
        Manhattan,
        Euclidean,
        Goober,
        Andradian
    }

    /// <summary>
    /// Represents a navigational graph used for A* pathfinding over Octree-based nodes.
    /// </summary>
    public class OctNavGraph
    {
        public readonly Dictionary<OctNode, GraphNode> nodes = new Dictionary<OctNode, GraphNode>();
        public readonly HashSet<GraphEdge> edges = new HashSet<GraphEdge>();

        private readonly Dictionary<OctNode, OctVolume> nodeToVolumeMap = new Dictionary<OctNode, OctVolume>();

        public int maxIterations = 500; //500 is alot we should see numbers from 30-200
        public HeuristicType heuristicType;
        int count = 0;

        /// <summary>
        /// Represents a node in the A* graph, which wraps an OctNode and includes A* metadata.
        /// </summary>
        public class GraphNode
        {
            static int nextId;
            public readonly int id;

            public float f, g, h;
            public GraphNode from;

            /// <summary>
            /// Gets the vector from the 'from' node to this node, used for directional heuristics.
            /// </summary>
            public Vector3 fromDirection => from != null ? center - from.center : Vector3.zero;

            public bool isHitNode;
            public List<GraphEdge> edges = new List<GraphEdge>();

            public OctNode octreeNode;

            /// <summary>
            /// Gets the center position of the node, accounting for edge direction if applicable.
            /// </summary>
            public Vector3 center
            {
                get
                {
                    Vector3 _center = octreeNode.bounds.center;

                    if (isHitNode)
                    {
                        Vector3 ext = octreeNode.bounds.extents;

                        _center.y += ext.y;
                        if (octreeNode.isEdge)
                        {
                            foreach (Direction dir in octreeNode.edgeDirs)
                            {
                                switch (dir)
                                {
                                    case Direction.PosX: _center.x += ext.x; break;
                                    case Direction.NegX: _center.x -= ext.x; break;
                                    case Direction.PosZ: _center.z += ext.z; break;
                                    case Direction.NegZ: _center.z -= ext.z; break;
                                }
                            }
                        }
                    }

                    return _center;
                }
            }

            /// <summary>
            /// Gets the bounds of the node centered around its adjusted center.
            /// </summary>
            public Bounds bounds
            {
                get
                {
                    Vector3 _center = center;

                    Vector3 _size = octreeNode.bounds.size;

                    return new Bounds(_center, _size);
                }
            }

            /// <summary>
            /// Gets the size of the node's bounding volume.
            /// </summary>
            public Vector3 size => octreeNode.bounds.size;

            public GraphNode(OctNode octreeNode, bool isHitNode = false)
            {
                this.id = nextId++;
                this.octreeNode = octreeNode;
                this.isHitNode = isHitNode;
            }

            public override bool Equals(object obj)
            {
                return obj is GraphNode other && other.id == id;
            }

            public override int GetHashCode()
            {
                return id;
            }
        }

        /// <summary>
        /// Represents an undirected edge between two graph nodes.
        /// </summary>
        public class GraphEdge
        {
            public readonly GraphNode a, b;

            public GraphEdge(GraphNode a, GraphNode b)
            {
                this.a = a;
                this.b = b;
            }

            public override bool Equals(object obj)
            {
                return obj is GraphEdge other && ((a == other.a && b == other.b) || (a == other.b && b == other.a));
            }

            public override int GetHashCode()
            {
                return a.GetHashCode() ^ b.GetHashCode();
            }
        }

        public List<GraphNode> AStar(OctNode startNode, OctNode endNode, HeuristicType heuristic)
        {
            heuristicType = heuristic; 
            count = 0;
            GraphNode start = FindNode(startNode);
            GraphNode end = FindNode(endNode);

 /*           DrawBounds2D(startNode.bounds,Color.green);
            DrawBounds2D(endNode.bounds,Color.cyan);*/
            if (start == null || end == null)
            {
                Debug.LogWarning("Start or End node not found in the graph.");
                return null;
            }
            foreach (GraphNode node in nodes.Values)
            {
                node.f = float.PositiveInfinity;
                node.g = float.PositiveInfinity;
                node.h = 0f;
                node.from = null;
            }

            SortedSet<GraphNode> openSet = new SortedSet<GraphNode>(new NodeComparer());
            HashSet<GraphNode> closedSet = new HashSet<GraphNode>();

            start.g = 0;
            start.h = Heuristic(start, end);    
            start.f = start.g + start.h;

            GraphNode bestReached = start;
            float bestH = Heuristic(start, end);
            openSet.Add(start);
            List<GraphNode> pathList = new List<GraphNode>();
            while (openSet.Count > 0)
            {
                if (++count > maxIterations)
                {
                  //  Debug.LogWarning($"A* exceeded maximum iterations, utilizing fallback. Iterations{maxIterations}");
                    break;
                }

                GraphNode current = openSet.First();
                openSet.Remove(current);

                if (current.h < bestH)
                {
                    bestH = current.h;
                    bestReached = current;
                }

                if (current.Equals(end))
                {
                    ReconstructPath(pathList, current);
                    return pathList;

                }

                closedSet.Add(current);

                foreach (GraphEdge edge in current.edges)
                {
                    GraphNode neighbor = (edge.a == current) ? edge.b : edge.a;

                    if (closedSet.Contains(neighbor)) continue;
                    float tentativeG = current.g + Heuristic(current, neighbor);


                    if (tentativeG < neighbor.g)
                    {
                       // Debug.DrawLine(current.center, neighbor.center, Color.green, 10f);
                        neighbor.from = current;
                        neighbor.g = tentativeG;
                        neighbor.h = Heuristic(neighbor, end);
                        neighbor.f = neighbor.g + neighbor.h;

                        openSet.Remove(neighbor);  // always remove/re-add to refresh order
                        openSet.Add(neighbor);
                    }
                }
            }
            List<GraphNode> fallbackPath = new List<GraphNode>();
            ReconstructPath(fallbackPath, bestReached);
            return fallbackPath.Count <2 ? null : fallbackPath; // fallback path since a lot of the reason why it fails is that its too far from the target
        }

        /// <summary>
        /// Reconstructs the path from end node back to start.
        /// </summary>
        private void ReconstructPath(List<GraphNode> pathList, GraphNode current)
        {
            while (current != null)
            {
                pathList.Add(current);
                current = current.from;
            }

            pathList.Reverse();
        }

        /// <summary>
        /// Computes the heuristic value between two nodes based on selected heuristic type.
        /// </summary>
        private float Heuristic(GraphNode a, GraphNode b)
        {
            Vector3 centerA = a.center;
            Vector3 centerB = b.center;
         
            float dx = Mathf.Abs(centerA.x - centerB.x);
            float dy = Mathf.Abs(centerA.y - centerB.y);
            float dz = Mathf.Abs(centerA.z - centerB.z);

            float centerDistance = Mathf.Sqrt(dx * dx + dy * dy + dz * dz);
            Vector3 diff = centerB - centerA;
          
            switch (heuristicType)
            {
                case HeuristicType.Morrisium:
                    float dnx = Mathf.Abs(diff.x) - (a.size.x * 0.5f + b.size.x * 0.5f);
                    float dny = Mathf.Abs(diff.y) - (a.size.y * 0.5f + b.size.y * 0.5f);
                    float dnz = Mathf.Abs(diff.z) - (a.size.z * 0.5f + b.size.z * 0.5f);
                    dnx = Mathf.Max(0f, dnx);
                    dny = Mathf.Max(0f, dny);
                    dnz = Mathf.Max(0f, dnz);
                    return (dnx + dny + dnz);

                case HeuristicType.Manhattan:
                    return (dx + dy + dz) * (dx + dy + dz);

                case HeuristicType.Euclidean:
                    return (dx * dx + dy * dy + dz * dz);                
                case HeuristicType.Goober:
                    return  (a.bounds.center - b.bounds.ClosestPoint(a.bounds.center)).sqrMagnitude;
              /*  case HeuristicType.Straight:
                    
                    Vector3 prevDir = a.fromDirection.normalized;
                    Vector3 nextDir = (b.center - a.center).normalized;
                  
                    float rawBend = (prevDir - nextDir).sqrMagnitude;
                    float inverted = 1f / (rawBend  + 1f);
                    return inverted * (dx * dx + dy * dy + dz * dz);*/
                case HeuristicType.Andradian:
                    float directionalWeight = ( (a.fromDirection - b.fromDirection).sqrMagnitude)+ 1;
                return directionalWeight * (dx * dx + dy * dy + dz * dz);

                default:
                    return (dx * dx + dy * dy + dz * dz);

            }
        }

        /// <summary>
        /// Used by the open set to compare nodes based on their f score.
        /// </summary>
        public class NodeComparer : IComparer<GraphNode>
        {
            public int Compare(GraphNode x, GraphNode y)
            {
                if (x == null || y == null) return 0;

                int compare = x.f.CompareTo(y.f);
                if (compare == 0)
                {
                    return x.id.CompareTo(y.id);
                }
                return compare;
            }
        }

        /// <summary>
        /// Adds a node to the graph, optionally marking it as a hit node.
        /// </summary>
        public void AddNode(OctVolume volume, OctNode octreeNode, bool isHitNode = false)
        {
            if (!nodes.ContainsKey(octreeNode))
            {
                nodes[octreeNode] = new GraphNode(octreeNode, isHitNode);
            }

            // Always update volume map
            nodeToVolumeMap[octreeNode] = volume;
        }

        /// <summary>
        /// Adds a bidirectional edge between two nodes in the graph.
        /// </summary>
        public void AddEdge(OctNode a, OctNode b)
        {
            GraphNode nodeA = FindNode(a);
            GraphNode nodeB = FindNode(b);

            if (nodeA == null || nodeB == null) return;

            GraphEdge edge = new GraphEdge(nodeA, nodeB);
            if (edges.Add(edge))
            {
                nodeA.edges.Add(edge);
                nodeB.edges.Add(edge);
            }
        }

        /// <summary>
        /// Removes all nodes and edges associated with the given volume.
        /// </summary>
        public void ClearVolume(OctVolume volume)
        {
            // Find all nodes belonging to this volume
            List<OctNode> nodesToRemove = nodeToVolumeMap
                .Where(kvp => kvp.Value == volume)
                .Select(kvp => kvp.Key)
                .ToList();

            // Remove edges connected to these nodes
            foreach (OctNode octNode in nodesToRemove)
            {
                if (nodes.TryGetValue(octNode, out GraphNode graphNode))
                {
                    // Remove edges from both nodes and global edge set
                    foreach (GraphEdge edge in graphNode.edges.ToList())
                    {
                        edges.Remove(edge);

                        if (edge.a != graphNode)
                        { 
                            edge.a.edges.Remove(edge); 
                        }
                        if (edge.b != graphNode)
                        {
                            edge.b.edges.Remove(edge);
                        }
                    }

                    graphNode.edges.Clear();
                }

                nodes.Remove(octNode);
                nodeToVolumeMap.Remove(octNode);
            }
        }

        /// <summary>
        /// Draws the graph nodes and edges using Gizmos.
        /// </summary>
        public void DrawGraphGizmos()
        {
            foreach (GraphEdge edge in edges)
            {
                Gizmos.DrawLine(edge.a.center, edge.b.center);
            }
            foreach (GraphNode node in nodes.Values)
            {
                Gizmos.DrawWireSphere(node.center,0.1f);
            }
        }

        /// <summary>
        /// Finds and returns the graph node associated with the given OctNode.
        /// </summary>
        public GraphNode FindNode(OctNode octreeNode)
        {
            nodes.TryGetValue(octreeNode, out GraphNode node);
            return node;
        }

        /// <summary>
        /// Finds the closest OctNode in the graph to a given position.
        /// </summary>
        public OctNode GetClosestNode(Vector3 position)
        {
            OctNode closestNode = null;
            float closestDistanceSqr = Mathf.Infinity;

            foreach (var nodePair in nodes)
            {
                OctNode node = nodePair.Key;

                float distanceSqr = (node.bounds.ClosestPoint(position) - position).sqrMagnitude;
                if (distanceSqr < closestDistanceSqr)
                {
                    closestDistanceSqr = distanceSqr;
                    closestNode = node;
                }
            }

            return closestNode;
        }
    }
}