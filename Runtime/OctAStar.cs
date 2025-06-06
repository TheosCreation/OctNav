using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace OctNav {
    public class Node
    {
        static int nextId;
        public readonly int id;

        public float f, g, h; //costs
        public Node from;

        public List<Edge> edges = new List<Edge>();

        public OctNode octreeNode;

        public Node(OctNode octreeNode)
        {
            this.id = nextId++;
            this.octreeNode = octreeNode;
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
        public int maxIterations = 1000;

        int count = 0;
        public List<Node> AStar(OctNode startNode, OctNode endNode, bool groundOnly = false, float maxStepHeight = 0.5f)
        {
            count = 0;
            Node start = FindNode(startNode);
            Node end = FindNode(endNode);
            if (start == null || end == null)
            {
                Debug.LogError("Start or End node not found in the graph.");
                return null;
            }
            foreach (var node in nodes.Values)
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

            openSet.Add(start);
            List<Node> pathList = new List<Node>();
            while (openSet.Count > 0)
            {
                if (++count > maxIterations)
                {
                    Debug.LogError("A* exceeded maximum iterations.");
                    return null;
                }

                Node current = openSet.First();
                openSet.Remove(current);

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
                    if (groundOnly)
                    {
                        float currentY = current.octreeNode.bounds.center.y;
                        float neighborY = neighbor.octreeNode.bounds.center.y;

                        if (Mathf.Abs(neighborY - currentY) > maxStepHeight)
                        {
                            continue;
                        }
                    }
                    float tentativeG = current.g + Heuristic(current, neighbor);

                    if (tentativeG < neighbor.g)
                    {
                        neighbor.from = current;
                        neighbor.g = tentativeG;
                        neighbor.h = Heuristic(neighbor, end);
                        neighbor.f = neighbor.g + neighbor.h;

                        openSet.Remove(neighbor);  // always remove/re-add to refresh order
                        openSet.Add(neighbor);
                    }
                }
            }

            Debug.Log("No Path found.");
            return null;

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
            return (a.octreeNode.bounds.center - b.octreeNode.bounds.center).sqrMagnitude;
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

        public void AddNode(OctNode octreeNode)
        {
            if (!nodes.ContainsKey(octreeNode))
            {
                nodes.Add(octreeNode, new Node(octreeNode));
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
        public void DrawGraph()
        {
            Gizmos.color = Color.cyan;
            foreach (Edge edge in edges)
            {
                Gizmos.DrawLine(edge.a.octreeNode.bounds.center, edge.b.octreeNode.bounds.center);
            }
            foreach (Node node in nodes.Values)
            {
                Gizmos.DrawWireSphere(node.octreeNode.bounds.center,0.1f);
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

                float distanceSqr = (node.bounds.center - position).sqrMagnitude;
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