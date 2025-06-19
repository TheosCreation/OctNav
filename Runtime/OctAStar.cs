using System.Collections.Generic;
using System.Linq;
using Unity.VisualScripting.YamlDotNet.Serialization.NodeDeserializers;
using UnityEngine;

namespace OctNav
{
    public enum HeuristicType
    {

        Morrisium,
        Manhattan,
        Euclidean,
        Goober,
        Andradian
    }
    public class Node
    {
        static int nextId;
        public readonly int id;

        public float f, g, h; //costs
        public Node from;

        public Vector3 fromDirection => from != null ? center - from.center : Vector3.zero;
        public bool isHitNode;
        public List<Edge> edges = new List<Edge>();

        public OctNode octreeNode;
        public Vector3 center
        {
            get
            {
                Vector3 _center = octreeNode.bounds.center;

                if (isHitNode)
                {
                    Vector3 ext = octreeNode.bounds.extents;

                    _center.y += ext.y;
                    if (octreeNode.isEdge) { 
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

        public Bounds bounds
        {
            get
            {
                Vector3 _center = center;

                Vector3 _size = octreeNode.bounds.size;

                return new Bounds(_center, _size);
            }
        }
        public Vector3 size => octreeNode.bounds.size;
        public Node(OctNode octreeNode, bool isHitNode = false)
        {
            this.id = nextId++;
            this.octreeNode = octreeNode;
            this.isHitNode = isHitNode;
        }
        public override bool Equals(object obj)
        {
            return obj is Node other && other.id == id;
        }
        public override int GetHashCode()
        {
            return id;
        }
    }

    public class Edge
    {
        public readonly Node a, b;

        public Edge(Node a, Node b)
        {
            this.a = a;
            this.b = b;
        }

        public override bool Equals(object obj)
        {
            return obj is Edge other && ((a == other.a && b == other.b) || (a == other.b && b == other.a));
        }

        public override int GetHashCode()
        {
            return a.GetHashCode() ^ b.GetHashCode();
        }
    }


    public class Graph
    {
        public readonly Dictionary<OctNode, Node> nodes = new Dictionary<OctNode, Node>();
        public readonly HashSet<Edge> edges = new HashSet<Edge>();

        private readonly Dictionary<OctNode, OctVolume> nodeToVolumeMap = new Dictionary<OctNode, OctVolume>();

        public int maxIterations = 500; //500 is alot we should see numbers from 30-200
        public HeuristicType heuristicType;
        int count = 0;
      //  public Graph(HeuristicType heuristicType = HeuristicType.SqrMagnitude) => this.heuristicType = heuristicType;
      
        public List<Node> AStar(OctNode startNode, OctNode endNode, HeuristicType heuristic)
        {
            heuristicType = heuristic; 
            count = 0;
            Node start = FindNode(startNode);
            Node end = FindNode(endNode);

 /*           DrawBounds2D(startNode.bounds,Color.green);
            DrawBounds2D(endNode.bounds,Color.cyan);*/
            if (start == null || end == null)
            {
                Debug.LogWarning("Start or End node not found in the graph.");
                return null;
            }
            foreach (Node node in nodes.Values)
            {
                node.f = float.PositiveInfinity;
                node.g = float.PositiveInfinity;
                node.h = 0f;
                node.from = null;
            }

            SortedSet<Node> openSet = new SortedSet<Node>(new NodeComparer());
            HashSet<Node> closedSet = new HashSet<Node>();

            start.g = 0;
            start.h = Heuristic(start, end);    
            start.f = start.g + start.h;

            Node bestReached = start;
            float bestH = Heuristic(start, end);
            openSet.Add(start);
            List<Node> pathList = new List<Node>();
            while (openSet.Count > 0)
            {
                if (++count > maxIterations)
                {
                  //  Debug.LogWarning($"A* exceeded maximum iterations, utilizing fallback. Iterations{maxIterations}");
                    break;
                }

                Node current = openSet.First();
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

                foreach (Edge edge in current.edges)
                {
                    Node neighbor = (edge.a == current) ? edge.b : edge.a;

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
            List<Node> fallbackPath = new List<Node>();
            ReconstructPath(fallbackPath, bestReached);
            return fallbackPath.Count <2 ? null : fallbackPath; // fallback path since a lot of the reason why it fails is that its too far from the target
        }


        private void ReconstructPath(List<Node> pathList, Node current)
        {
            while (current != null)
            {
                pathList.Add(current);
                current = current.from;
            }

            pathList.Reverse();
        }

        private float Heuristic(Node a, Node b)
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

        //private bool IsPathClear(Vector3 from, Vector3 to, float radius)
        //{
        //    Vector3 direction = to - from;
        //    float distance = direction.magnitude;
        //    direction.Normalize();
        //
        //    // Adjust this layerMask as per your scene setup
        //    int layerMask = LayerMask.GetMask("Environment");
        //
        //    return !Physics.SphereCast(from, radius, direction, out RaycastHit hit, distance, layerMask);
        //}

        public class NodeComparer : IComparer<Node>
        {
            public int Compare(Node x, Node y)
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

        public void AddNode(OctVolume volume, OctNode octreeNode, bool isHitNode = false)
        {
            if (!nodes.ContainsKey(octreeNode))
            {
                nodes.Add(octreeNode, new Node(octreeNode, isHitNode));
                nodeToVolumeMap[octreeNode] = volume;
            }
        }

        public void AddEdge(OctNode a, OctNode b)
        {
            Node nodeA = FindNode(a);
            Node nodeB = FindNode(b);

            if (nodeA == null || nodeB == null) return;

            Edge edge = new Edge(nodeA, nodeB);
            if (edges.Add(edge))
            {
                nodeA.edges.Add(edge);
                nodeB.edges.Add(edge);
            }
        }

        public void ClearVolume(OctVolume volume)
        {
            // Find all nodes belonging to this volume
            var nodesToRemove = nodeToVolumeMap.Where(kvp => kvp.Value == volume)
                                              .Select(kvp => kvp.Key)
                                              .ToList();

            // Remove edges connected to these nodes
            foreach (var node in nodesToRemove)
            {
                if (nodes.TryGetValue(node, out var graphNode))
                {
                    // Remove all edges connected to this node
                    foreach (var edge in graphNode.edges.ToList())
                    {
                        edges.Remove(edge);
                        Node otherNode = edge.a == graphNode ? edge.b : edge.a;
                        otherNode.edges.Remove(edge);
                    }
                }

                nodes.Remove(node);
                nodeToVolumeMap.Remove(node);
            }
        }

        public void DrawGraph()
        {
            
            foreach (Edge edge in edges)
            {
                Gizmos.DrawLine(edge.a.center, edge.b.center);
            }
            foreach (Node node in nodes.Values)
            {
                Gizmos.DrawWireSphere(node.center,0.1f);
            }
        }

        public Node FindNode(OctNode octreeNode)
        {
            nodes.TryGetValue(octreeNode, out Node node);
            return node;
        }

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