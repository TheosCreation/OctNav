using System.Collections.Generic;
using System.Linq;
using Unity.VisualScripting;
using UnityEngine;
using Random = UnityEngine.Random;

namespace OctNav
{
    public class AgentPath
    {
        public List<Vector3> waypoints = new List<Vector3>();

        public void Clear()
        {
            waypoints.Clear();
        }

        public void Add(Vector3 point)
        {
            waypoints.Add(point);
        }

        public int length => waypoints.Count;
        public float travelDistance { get
            {
                float currentDist = 0f;
                Vector3 lastPoint = waypoints[0];
                for (int i = 1; i<waypoints.Count; i++) 
                {
                    currentDist += Vector3.Distance(lastPoint, waypoints[i]);
                    lastPoint = waypoints[i];
                }
                return currentDist;
            } 
        }
        

        public Vector3 this[int i] => waypoints[i];
    }

    public class OctreeNavigationAgent : MonoBehaviour
    {
        [Header("Base Locomotion")]
        public float baseSpeed = 5f;
        public float accuracy = 1f;
        public float turnSpeed = 5f;

        [Header("Path Smoothing")]
        public bool enableSmoothing = true;
        public float agentRadius = 0.5f;
        public LayerMask geometryMask;
        public int maxSmoothingSteps = 5;
        public bool useLookAhead = false;

        [Header("Octree Information")]
        public Transform target;

        [Header("Turn-Rate Clamping")]
        public float maxTurnRate = 5f;

        [Header("Speed Curves")]
        public AnimationCurve speedOverDistance = AnimationCurve.Linear(0, 1, 1, 1);
        public AnimationCurve speedOverTurning = AnimationCurve.Linear(0, 1, 5, 0.5f);

        [Header("Ground-Only Settings")]
        public bool groundOnly = false;
        public float maxStepHeight = 0.5f;

        List<Node> currentAStarPath = new List<Node>();
        public AgentPath currentPath = new AgentPath();

        int currentWaypoint;
        Vector3 lastForward;
        OctNode currentNode;

        private bool waitingForNewPath = false;
        private bool forceNewPath = false;
        private bool hasValidPath = false;
        private int lookAheadSteps = 2;

        private void Start()
        {
            currentNode = OctNavigation.graph.GetClosestNode(transform.position);
            lastForward = transform.forward;

            if (target != null) PathToTarget();
            else GetRandomDestination();
        }

        private void Update()
        {
            if (OctNavigation.graph == null) return;
            foreach (var p in portals)
            {
                for (int i = 0; i < 4; i++)
                    Debug.DrawLine(p[i], p[(i + 1) % 4], Color.magenta, 5f);
            }
            if (forceNewPath)
            {
                forceNewPath = false;
                SchedulePathRefresh();
                return;
            }

            if (currentAStarPath.Count == 0 || currentWaypoint >= currentAStarPath.Count)
            {
                if (!hasValidPath || currentWaypoint >= currentAStarPath.Count)
                {
                    if (!waitingForNewPath)
                    {
                        waitingForNewPath = true;
                        Invoke(nameof(GetNewPath), 0.1f);
                        SchedulePathRefresh();
                    }
                    return;
                }
            }

            waitingForNewPath = false;

            if (currentWaypoint >= GetPathLength()) return;

            Vector3 nextPos = currentPath[currentWaypoint];

            float distTo = Vector3.Distance(transform.position, nextPos);

            if (distTo < accuracy)
            {
                currentWaypoint++;
                return;
            }

            if (groundOnly)
            {
                float dy = nextPos.y - transform.position.y;
                if (Mathf.Abs(dy) > maxStepHeight)
                {
                    hasValidPath = false;
                    forceNewPath = true;
                    return;
                }
            }

            float percent = (float)currentWaypoint / (GetPathLength() - 1);
            float modDist = speedOverDistance.Evaluate(percent);

            Vector3 desiredDir = (nextPos - transform.position).normalized;
            float angle = Vector3.Angle(lastForward, desiredDir) * Mathf.Deg2Rad;
            float turnRate = angle / Time.deltaTime;
            float modTurn = speedOverTurning.Evaluate(Mathf.Min(turnRate, maxTurnRate));

            float finalSpeed = baseSpeed * modDist * modTurn;

            transform.rotation = Quaternion.Slerp(
                transform.rotation,
                Quaternion.LookRotation(desiredDir, Vector3.up),
                turnSpeed * Time.deltaTime
            );

            transform.position = Vector3.MoveTowards(
                transform.position,
                nextPos,
                finalSpeed * Time.deltaTime
            );

            lastForward = transform.forward;
        }

        private Vector3 GetAdjustedCenter(int index)
        {
            OctNode current = GetPathNode(index);
            OctNode next = GetPathNode(index + 1 < GetPathLength() ? index + 1 : index);
            Vector3 dir = (next.bounds.center - current.bounds.center).normalized;
            return current.bounds.center + Vector3.Scale(dir, current.bounds.extents * 0.5f);
        }

        private void SchedulePathRefresh()
        {
            CancelInvoke(nameof(GetNewPath));
            Invoke(nameof(GetNewPath), 0.1f);
        }

        public int GetPathLength() => currentAStarPath?.Count ?? 0;

        public OctNode GetPathNode(int index)
        {
            if (index < 0 || index >= currentAStarPath.Count)
            {
                Debug.LogError($"Index out of bounds. Path length: {currentAStarPath.Count}, Index: {index}");
                return null;
            }
            return currentAStarPath[index].octreeNode;
        }

        private void GetNewPath()
        {
            waitingForNewPath = false;
            if (OctNavigation.graph == null || OctNavigation.graph.nodes.Count == 0)
            {
                Debug.LogError("Graph is null or contains no nodes.");
                hasValidPath = false;
                return;
            }

            currentNode = OctNavigation.graph.GetClosestNode(transform.position);

            if (target != null) PathToTarget();
            else GetRandomDestination();
        }
        private AgentPath BuildLookaheadPath(List<Node> nodePath, Vector3 targetPosition )
        {
            AgentPath path = new AgentPath();
            if (nodePath == null || nodePath.Count == 0) return path;

            int nodeCount = nodePath.Count;
            Vector3 previousPos = transform.position;

            for (int i = 1; i < nodeCount; i++)
            {
                Node midNode = nodePath[i];
                Bounds midBounds = midNode.octreeNode.bounds;
                //shrink to fat agent radius
                Bounds shrunk = new Bounds(midBounds.center, midBounds.size);
                shrunk.Expand(-2f * agentRadius);
                if (shrunk.size.x <= 0f || shrunk.size.y <= 0f || shrunk.size.z <= 0f)
                {
                    shrunk = midBounds;
                }

                int lookaheadIndex = i + lookAheadSteps;
                Vector3 waypoint;

                if (lookaheadIndex < nodeCount)
                {
                    // try to look ahead
                    Vector3 lookaheadCenter = nodePath[lookaheadIndex].octreeNode.bounds.center;
                    Vector3 direction = (lookaheadCenter - previousPos).normalized;

                    Ray r = new Ray(previousPos, direction);
                    if (shrunk.IntersectRay(r, out float dist))
                    {

                        waypoint = r.GetPoint(dist);
                    }
                    else
                    {
                        waypoint = shrunk.ClosestPoint(lookaheadCenter);
                    }
                }
                else
                {
                    Vector3 direction = (targetPosition - previousPos).normalized;
                    Ray r = new Ray(previousPos, direction);
                    if (shrunk.IntersectRay(r, out float dist))
                    {
                        waypoint = r.GetPoint(dist);
                    }
                    else
                    {
                        waypoint = shrunk.ClosestPoint(targetPosition);
                    }
                }

                path.Add(waypoint);
                previousPos = waypoint;
            }

            path.Add(targetPosition);
            return path;
        }

        private void PathToTarget()
        {
            if (target == null || OctNavigation.graph == null || OctNavigation.graph.nodes.Count == 0)
            {
                Debug.LogError("Target or graph is null.");
                hasValidPath = false;
                return;
            }

            currentNode = OctNavigation.graph.GetClosestNode(transform.position);
            OctNode targetNode = OctNavigation.graph.GetClosestNode(target.position);

            if (currentNode == targetNode)
            {
                currentAStarPath = new List<Node> { OctNavigation.graph.FindNode(currentNode) };
                currentPath.Clear();
                currentPath.Add(target.position);
                currentWaypoint = 0;
                hasValidPath = true;
                return;
            }

            currentAStarPath = OctNavigation.graph.AStar(currentNode, targetNode, groundOnly, maxStepHeight);
            if (currentAStarPath == null)
            {
                Debug.LogWarning("PathToTarget failed: no valid path found.");
                hasValidPath = false;
                forceNewPath = true;
                return;
            }

            currentWaypoint = 0;
            hasValidPath = true;
            currentPath = BuildSmoothedPath(currentAStarPath);

            //string s = "A* leaf centers: ";
            //for (int i = 0; i < currentAStarPath.Count; i++)
            //{
            //    Vector3 c = currentAStarPath[i].octreeNode.bounds.center;
            //    s += $"({c.x:F1},{c.y:F1},{c.z:F1}) ";
            //}
            //Debug.Log(s);
        }
        List<List<Vector3>> portals;
        public AgentPath BuildSmoothedPath(List<Node> rawPath)
        {
            AgentPath path = new AgentPath();
            if (rawPath == null || rawPath.Count < 2)
            {
                if (rawPath != null)
                    foreach (var n in rawPath)
                        path.Add(n.octreeNode.bounds.center);
                return path;
            }

            List<Vector3> centers = rawPath.Select(n => n.octreeNode.bounds.center).ToList();
            portals = ExtractPortals(rawPath);
            return Funnel3DPath(centers, portals);
        }

        public AgentPath Funnel3DPath(List<Vector3> pathPoints, List<List<Vector3>> portals)
        {
            AgentPath path = new AgentPath();
            path.Add(pathPoints[0]); // Start at the first node center

            Vector3 apex = pathPoints[0];
            Vector3 left = apex;
            Vector3 right = apex;
            int apexIndex = 0;
            int leftIndex = 0;
            int rightIndex = 0;

            for (int i = 0; i < portals.Count; i++)
            {
                var (newLeft, newRight) = GetPortalEdge(apex, portals[i]);

                // tighten right
                if (IsLeftOf(apex, right, newRight))
                {
                    right = newRight;
                    rightIndex = i + 1;
                }
                else
                {
                    // funnel collapsed on right
                    path.Add(left);
                    apex = left;
                    apexIndex = leftIndex;
                    i = apexIndex;
                    left = apex;
                    right = apex;
                    leftIndex = apexIndex;
                    rightIndex = apexIndex;
                    continue;
                }

                // tighten left
                if (IsRightOf(apex, left, newLeft))
                {
                    left = newLeft;
                    leftIndex = i + 1;
                }
                else
                {
                    // funnel collapsed on left
                    path.Add(right);
                    apex = right;
                    apexIndex = rightIndex;
                    i = apexIndex;
                    left = apex;
                    right = apex;
                    leftIndex = apexIndex;
                    rightIndex = apexIndex;
                    continue;
                }
            }

            // Add final point
            path.Add(pathPoints[^1]);
            return path;
        }

        private (Vector3 left, Vector3 right) GetPortalEdge(Vector3 apex, List<Vector3> quad)
        {
            // Find horizontal (XZ) apex & portal center
            Vector3 apexXZ = new Vector3(apex.x, 0, apex.z);

            // If quad[0] and quad[2] are opposite corners, their average is the true center.
            Vector3 center3D = (quad[0] + quad[2]) * 0.5f;
            Vector3 centerXZ = new Vector3(center3D.x, 0, center3D.z);

            // Compute funnel‐axis direction in XZ
            Vector3 forwardDirXZ = (centerXZ - apexXZ).normalized;
            if (forwardDirXZ == Vector3.zero)
            {
                // If apex is exactly at center, pick any direction (arbitrary)
                forwardDirXZ = Vector3.forward;
            }

            // Define a 2D “right” axis in XZ (perp to forwardDirXZ)
            Vector3 rightAxis = Vector3.Cross(Vector3.up, forwardDirXZ).normalized;
            // This rightAxis lies in XZ-plane, perpendicular to forwardDirXZ.

            // For each of the 4 corners, project to XZ and compute dot with rightAxis:
            float minDot = float.PositiveInfinity;
            float maxDot = float.NegativeInfinity;
            Vector3 leftCorner = quad[0];
            Vector3 rightCorner = quad[0];

            foreach (Vector3 corner in quad)
            {
                Vector3 cornerXZ = new Vector3(corner.x, 0, corner.z);
                float d = Vector3.Dot(cornerXZ - apexXZ, rightAxis);

                if (d < minDot)
                {
                    minDot = d;
                    leftCorner = corner;
                }
                if (d > maxDot)
                {
                    maxDot = d;
                    rightCorner = corner;
                }
            }

            return (leftCorner, rightCorner);
        }

        bool IsLeftOf(Vector3 apex, Vector3 edge, Vector3 test)
        {
            Vector3 a = edge - apex;
            Vector3 b = test - apex;
            return Vector3.Cross(a, b).y >= 0; // adjust axis if needed
        }

        bool IsRightOf(Vector3 apex, Vector3 edge, Vector3 test)
        {
            Vector3 a = edge - apex;
            Vector3 b = test - apex;
            return Vector3.Cross(a, b).y <= 0; // adjust axis if needed
        }
        public List<List<Vector3>> ExtractPortals(List<Node> path)
        {
            List<List<Vector3>> portals = new();

            for (int i = 0; i < path.Count - 1; i++)
            {
                var a = path[i].octreeNode.bounds;
                var b = path[i + 1].octreeNode.bounds;
                var portal = GetTouchingPortal(a, b);

                if (portal != null && portal.Count == 4)
                    portals.Add(portal);
                else
                    Debug.LogWarning($"Could not compute portal between node {i} and {i + 1}");
            }

            return portals;
        }
        public List<Vector3> GetTouchingPortal(Bounds a, Bounds b)
        {
            const float epsilon = 0.001f;

            Vector3 diff = b.center - a.center;
            Vector3 normal = Vector3.zero;

            // Determine the dominant axis of separation (shared face normal)
            if (Mathf.Abs(diff.x) > Mathf.Abs(diff.y) && Mathf.Abs(diff.x) > Mathf.Abs(diff.z))
                normal = Vector3.right * Mathf.Sign(diff.x);
            else if (Mathf.Abs(diff.y) > Mathf.Abs(diff.z))
                normal = Vector3.up * Mathf.Sign(diff.y);
            else
                normal = Vector3.forward * Mathf.Sign(diff.z);

            // Check if they are actually touching (not overlapping)
            float distance = Vector3.Dot(diff, normal);
            float expectedDistance = Mathf.Abs(Vector3.Dot(a.extents + b.extents, normal));

            if (Mathf.Abs(distance - expectedDistance) > epsilon)
                return null; // Not touching, just overlapping or separated

            // Get the touching plane position
            float planePos;
            if (normal.x != 0)
                planePos = (normal.x > 0) ? (a.max.x) : (a.min.x);
            else if (normal.y != 0)
                planePos = (normal.y > 0) ? (a.max.y) : (a.min.y);
            else
                planePos = (normal.z > 0) ? (a.max.z) : (a.min.z);

            // Get overlap in the other 2 axes
            Vector3 aMin = a.min, aMax = a.max;
            Vector3 bMin = b.min, bMax = b.max;

            List<Vector3> quad = new();

            if (normal.x != 0) // X-aligned portal
            {
                float yMin = Mathf.Max(aMin.y, bMin.y);
                float yMax = Mathf.Min(aMax.y, bMax.y);
                float zMin = Mathf.Max(aMin.z, bMin.z);
                float zMax = Mathf.Min(aMax.z, bMax.z);

                if (yMax - yMin < epsilon || zMax - zMin < epsilon)
                    return null; // No meaningful overlap

                quad.Add(new Vector3(planePos, yMin, zMin));
                quad.Add(new Vector3(planePos, yMin, zMax));
                quad.Add(new Vector3(planePos, yMax, zMax));
                quad.Add(new Vector3(planePos, yMax, zMin));
            }
            else if (normal.y != 0) // Y-aligned portal
            {
                float xMin = Mathf.Max(aMin.x, bMin.x);
                float xMax = Mathf.Min(aMax.x, bMax.x);
                float zMin = Mathf.Max(aMin.z, bMin.z);
                float zMax = Mathf.Min(aMax.z, bMax.z);

                if (xMax - xMin < epsilon || zMax - zMin < epsilon)
                    return null;

                quad.Add(new Vector3(xMin, planePos, zMin));
                quad.Add(new Vector3(xMax, planePos, zMin));
                quad.Add(new Vector3(xMax, planePos, zMax));
                quad.Add(new Vector3(xMin, planePos, zMax));
            }
            else if (normal.z != 0) // Z-aligned portal
            {
                float xMin = Mathf.Max(aMin.x, bMin.x);
                float xMax = Mathf.Min(aMax.x, bMax.x);
                float yMin = Mathf.Max(aMin.y, bMin.y);
                float yMax = Mathf.Min(aMax.y, bMax.y);

                if (xMax - xMin < epsilon || yMax - yMin < epsilon)
                    return null;

                quad.Add(new Vector3(xMin, yMin, planePos));
                quad.Add(new Vector3(xMax, yMin, planePos));
                quad.Add(new Vector3(xMax, yMax, planePos));
                quad.Add(new Vector3(xMin, yMax, planePos));
            }

            return quad;
        }


        void GetRandomDestination()
        {/*
            if (OctNavigation.graph.nodes.Count == 0)
            {
                Debug.LogWarning("No nodes available in graph.");
                return;
            }

            currentNode = OctNavigation.graph.GetClosestNode(transform.position);
            OctNode destinationNode;
            int tries = 0, maxTries = 20;

            do
            {
                destinationNode = OctNavigation.graph.nodes.ElementAt(Random.Range(0, OctNavigation.graph.nodes.Count)).Key;
                currentAStarPath = OctNavigation.graph.AStar(currentNode, destinationNode, groundOnly, maxStepHeight);
                if (currentAStarPath != null)
                {
                    currentWaypoint = 0;
                    hasValidPath = true;
                    currentPath = currentPath = useLookAhead ? BuildLookaheadPath(currentAStarPath, target.position) :
                OctPathSmoothing.BuildSmoothedPath(
                currentAStarPath,
                transform.position,
                target.position,
                agentRadius,
                maxSmoothingSteps
            );
                    return;
                }
                tries++;
            }
            while (tries < maxTries);

            Debug.LogWarning("GetRandomDestination failed: no path found after multiple attempts.");
            hasValidPath = false;
            forceNewPath = true;


            currentPath.Clear();
            for (int i = 0; i < currentAStarPath.Count - 1; i++)
            {
                Vector3 p1 = currentAStarPath[i].octreeNode.bounds.center;
                Vector3 p2 = currentAStarPath[i + 1].octreeNode.bounds.center;
                Vector3 dir = (p2 - p1).normalized;
                Vector3 offset = Vector3.Scale(dir, currentAStarPath[i].octreeNode.bounds.extents * 0.5f);
                currentPath.Add(p1 + offset);
            }
            // Add final point
            currentPath.Add(currentAStarPath[^1].octreeNode.bounds.center);*/
        }

        private void OnDrawGizmos()
        {
            if (OctNavigation.graph == null || GetPathLength() == 0) return;

            Gizmos.color = Color.red;
            Gizmos.DrawWireSphere(GetPathNode(0).bounds.center, 0.7f);

            Gizmos.color = Color.blue;
            Gizmos.DrawWireSphere(GetPathNode(GetPathLength() - 1).bounds.center, 0.7f);

            for (int i = 0; i < GetPathLength(); i++)
            {
                Gizmos.DrawWireSphere(GetPathNode(i).bounds.center, 0.5f);
                if (i < GetPathLength() - 1)
                {   
                    Gizmos.DrawLine(GetPathNode(i).bounds.center, GetPathNode(i + 1).bounds.center);
                }
            }

            if (target != null)
            {
                Gizmos.color = Color.magenta;
                Gizmos.DrawLine(transform.position, target.position);
            }

          
            if (currentPath.length > 0)
            {
                Gizmos.color = Color.yellow;
                for (int i = 0; i < currentPath.length; i++)
                {
                    Vector3 wp = currentPath[i];
                    Gizmos.DrawSphere(wp, 0.15f);

                    if (i < currentPath.length - 1)
                    {
                        Vector3 nextWp = currentPath[i + 1];
                        Gizmos.DrawLine(wp, nextWp);
                    }
                }
            }
         
        }

    }
}