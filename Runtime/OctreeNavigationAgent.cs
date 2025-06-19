using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.UIElements;
using static Codice.Client.Common.WebApi.WebApiEndpoints;

namespace OctNav
{
    public class AgentPath
    {
        public List<Vector3> waypoints = new List<Vector3>();
        public List<int> rawIndexMap = new List<int>();
        public void Clear()
        {
            waypoints.Clear();
        }

        public void Add(Vector3 point)
        {
            waypoints.Add(point);
        }


        public int length => waypoints.Count;
        public float travelDistance {
            get
            {
                if (waypoints.Count < 2) return 0f;
                float currentDist = 0f;
                Vector3 lastPoint = waypoints[0];
                for (int i = 1; i < waypoints.Count; i++)
                {
                    currentDist += Vector3.Distance(lastPoint, waypoints[i]);
                    lastPoint = waypoints[i];
                }
                return currentDist;
            }
        }
        public float ClosestDistanceToPoint(Vector3 point)
        {
            if (waypoints.Count < 2) return Vector3.Distance(point, waypoints.Count == 1 ? waypoints[0] : Vector3.zero);

            float minDistance = float.MaxValue;

            for (int i = 0; i < waypoints.Count - 1; i++)
            {
                Vector3 a = waypoints[i];
                Vector3 b = waypoints[i + 1];
                
                Vector3 ab = b - a;
                Vector3 ap = point - a;
                float t = Vector3.Dot(ap, ab) / ab.sqrMagnitude;
                // cosine of the angle from the ap and ab vectors https://www.desmos.com/calculator/mdifkrdpoy\
                // gets like index of closest point

                t = Mathf.Clamp01(t);
                //clamp it

                Vector3 closestPoint = a + t * ab;
                // just turns that index to the closest point
                float distance = Vector3.Distance(point, closestPoint);

                if (distance < minDistance)
                {
                    minDistance = distance;
                }
            }

            return minDistance;
        }
    
        public Vector3 GetDirection(int i)
        {
            if (i < 0 || i >= waypoints.Count - 1 || length<2)
            {
                return Vector3.zero;
            }
            Vector3 a = this[i];
            Vector3 b = this[i + 1];
            return (b-a).normalized;
        }

        public Vector3 this[int i]
        {
            get
            {
                if (i < 0 || i >= waypoints.Count)
                {
                    return waypoints.Count > 0 ? waypoints[waypoints.Count - 1] : Vector3.zero;
                }
                return waypoints[i];
            }
        }
    }

    [RequireComponent(typeof(Rigidbody))]
    public class OctreeNavigationAgent : MonoBehaviour
    {
        [Header("Base Locomotion")]
        public float maxSpeed = 5f;
        public float acceleration = 5f;
        public float accuracy = 1f;
        public float turnSpeed = 5f;
        public float stoppingDistance = 5f;
        public float deceleration = 5f;

        [Header("Path Recalculating")]
        [SerializeField] private float maxDistanceFromPath = 5f;
        [SerializeField] private float targetMoveThreshold = 0.1f;

        [Header("Path Smoothing")]
        bool enableSmoothing = true;
        public float agentRadius = 0.5f;
        public bool enableRaycastSmoothing = true;
        public int splineSubdivisionCount = 8;

        [Header("Dynamic Repathing")]
        public bool enableDynamicRepathing = true;
        public int nodesAwayBeforeRepath = 1;
        private bool isPreloadingNextPath = false;


        [Header("Octree Information")]
        public Transform target;
        public Vector3 currcurrgoal;

        [Header("Pathfinding Settings")]
        public HeuristicType heuristicType = HeuristicType.Euclidean;
        public bool walking = false;
        public bool straightPath = false;
        public bool stayUpright = true;
        public bool controlRotation = true;
        public float maxStepDistance = 1f;

        public AgentPath currentPath = new AgentPath();
        private List<Node> currentAStarPath = new List<Node>();

        [Header("Functionality")]
        public bool isPaused { get; private set; } = false;
        public bool isMoving { get; private set; } = false;
        private Vector3? manualDestination = null;
        public Vector3? currentGoal => manualDestination.HasValue ? manualDestination : target != null ? (Vector3?)target.position : null;
        public Vector3? nextGoal = null;
        public event Action OnDestinationReached;
        public event Action<List<Vector3>> OnPathUpdated;
        [Header("Utils")]
        public LayerMask layerMask = ~0;
        public bool rotationFrozen = false;

        [HideInInspector] public int currentWaypoint;

        private Vector3 lastForward;
        private OctNode currentNode;

        private bool forceNewPath = false;
        private bool reachedDestination = false;

        private List<List<Vector3>> portals = new List<List<Vector3>>();
        private List<Vector3> viaPoints = new List<Vector3>();

        private Rigidbody rb;
        private Vector3 currentVelocity;
        private Vector3 cashedVelocity;
        private Vector3 cashedAngularVelocity;
        private Vector3 targetCachedPosition;
        private bool isOnGround;
        private float groundCheckDistance = 0.2f;
        private float maxSlopeAngle = 30f;

        private void Start()
        {
            rb = GetComponent<Rigidbody>();
            rb.isKinematic = false;
            rb.interpolation = RigidbodyInterpolation.Interpolate;
            rb.collisionDetectionMode = CollisionDetectionMode.Continuous;
            if(walking)
            {
                rb.useGravity = true;
            }
            else
            {
                rb.useGravity = false;
            }

                currentNode = OctNavigation.GetGraph(walking).GetClosestNode(transform.position);
            lastForward = transform.forward;
            isPreloadingNextPath = false;
            forceNewPath = true;
        }
        private void FixedUpdate()
        {
            currcurrgoal = currentGoal ?? Vector3.zero;
            if (OctNavigation.GetGraph(walking) == null || currentGoal == null) return;
            GroundCheck();

            if (Vector3.Distance(targetCachedPosition, currentGoal.Value) > targetMoveThreshold && !straightPath)
            {
                isPreloadingNextPath = false;
                forceNewPath = true;
            }
            if (!reachedDestination && !isMoving)
            {
                reachedDestination = true;
                ApplyStoppingForce();
                OnDestinationReached?.Invoke();
            }
            if (!isMoving || isPaused)
            {
                ApplyStoppingForce();
                return;
            }

            /* if (stayUpright)
             {
                 Quaternion uprightRotation = Quaternion.LookRotation(transform.forward, Vector3.up);
                 rb.MoveRotation(uprightRotation);

             }*/
            float distToGoal = Vector3.Distance(transform.position, currentGoal.Value);
            if (distToGoal < stoppingDistance)
            {
                reachedDestination = true;
                ApplyStoppingForce();
                isMoving = false;
                OnDestinationReached?.Invoke();
                return;
            }

            Vector3 nextWaypointPosition = currentPath[currentWaypoint];
            float distToNextWaypoint = Vector3.Distance(transform.position, nextWaypointPosition);

            if (distToNextWaypoint < accuracy)
            {
                currentWaypoint++;
                if (currentWaypoint >= currentPath.length)
                {
                    currentWaypoint = currentPath.length - 1;
                }
            }
            if (!straightPath) { 
                if (forceNewPath)
                {
                    GetNewPath();
                    OnPathUpdated?.Invoke(currentPath.waypoints);
                    forceNewPath = false;
                }
                float pathDistance = PathDistance();
                if (pathDistance > maxDistanceFromPath)
                {

                    /*    if(pathDistance>maxDistanceFromPath*2 && currentWaypoint!=0)
                        {
                            Debug.Log("No suitable recovery waypoint found, recalculating path");
                            forceNewPath = true;
                        }
                        else { */
               
                    int bestWaypoint = FindBestNextWaypoint();
                    if (bestWaypoint != -1)
                    {
                        currentWaypoint = bestWaypoint;
                    }
                    else
                    {
                        Debug.Log("No suitable recovery waypoint found, recalculating path");
                        forceNewPath = true;
                        return;
                    }
                    //  }
                }
            }
            /*   if (straightPath && isMoving && currentVelocity.magnitude < 0.05f)
               {
                   stuckTimer += Time.fixedDeltaTime;
                   if (stuckTimer > 0.5f) // half-second of stuck time
                   {
                       Debug.LogWarning("Agent stuck mid-dash.");
                       ApplyStoppingForce();
                       isMoving = false;
                       reachedDestination = true;
                       OnDestinationReached?.Invoke();
                       return;
                   }
               }
               else
               {
                   stuckTimer = 0f;
               }*/

            if (currentPath.length == 0 || currentWaypoint >= currentPath.length)
            {
                if (!reachedDestination)
                {
                    reachedDestination = true;
                    OnDestinationReached?.Invoke();
                    Invoke(nameof(GetNewPath), 0.01f);
                }
                isMoving = false;
                return;
            }
            if (currentPath == null || currentPath.length == 0 || currentWaypoint < 0 || currentWaypoint >= currentPath.length)
            {
                forceNewPath = true;
                isPreloadingNextPath = false;
                isMoving = false;
                reachedDestination = true;
                return;
            }
            if (enableDynamicRepathing && nextGoal.HasValue)
            {
                int remaining =  currentAStarPath.Count - 1 - GetCurrentRawIndex();
                if (!isPreloadingNextPath && remaining < nodesAwayBeforeRepath && !straightPath && isMoving && !forceNewPath)
                {
                    PreloadAndMergePath();
                    isPreloadingNextPath = true;
                }
            }

            Vector3 directionVector = nextWaypointPosition - transform.position;
            Vector3 desiredDir = directionVector.normalized;

            // Calculate turn angle and turn speed modifier
            float angle = Vector3.Angle(lastForward, desiredDir);

            // Handle case where lastForward might be zero
            if (lastForward.magnitude < 0.001f)
            {
                lastForward = transform.forward;
                angle = Vector3.Angle(lastForward, desiredDir);
            }

            // Calculate how much the agent can actually turn this frame
            float maxTurnThisFrame = turnSpeed * Time.fixedDeltaTime;
            float actualTurnAngle = Mathf.Min(angle, maxTurnThisFrame);

            // Handle slopes
            if (walking && isOnGround)
            {
                RaycastHit hit;
                if (Physics.Raycast(transform.position, Vector3.down, out hit, groundCheckDistance * 2f))
                {
                    float slopeAngle = Vector3.Angle(hit.normal, Vector3.up);
                    if (slopeAngle > maxSlopeAngle)
                    {
                        // Project movement direction onto slope plane
                        Vector3 projectedDir = Vector3.ProjectOnPlane(desiredDir, hit.normal);
                        if (projectedDir.magnitude > 0.001f)
                        {
                            desiredDir = projectedDir.normalized;
                            //finalSpeed *= Mathf.Clamp01(1 - (slopeAngle - maxSlopeAngle) / (90f - maxSlopeAngle));
                        }
                        else
                        {
                            // If projection results in zero vector, stop movement
                            ApplyStoppingForce();
                            return;
                        }
                    }
                }
            }

            // Calculate movement direction based on turn speed constraints
            Vector3 movementDir;
            if (angle > maxTurnThisFrame && angle > 0.001f)
            {
                // If we can't turn fast enough, move in the direction we can actually face
                movementDir = Vector3.Slerp(lastForward, desiredDir, actualTurnAngle / angle);
            }
            else
            {
                movementDir = desiredDir;
            }

            // Validate movementDir
            if (float.IsNaN(movementDir.x) || float.IsNaN(movementDir.y) || float.IsNaN(movementDir.z))
            {
                Debug.LogWarning("movementDir is NaN, using desiredDir");
                movementDir = desiredDir;
            }

            // Calculate movement
            Vector3 targetVelocity = movementDir * maxSpeed;

            // Validate targetVelocity
            if (float.IsNaN(targetVelocity.x) || float.IsNaN(targetVelocity.y) || float.IsNaN(targetVelocity.z))
            {
                Debug.LogWarning("targetVelocity is NaN, stopping movement");
                return;
            }

            isMoving = true;

            Vector3 difference = targetVelocity - currentVelocity;
            currentVelocity += difference * (Time.fixedDeltaTime * acceleration);

            // Final validation of currentVelocity
            if (float.IsNaN(currentVelocity.x) || float.IsNaN(currentVelocity.y) || float.IsNaN(currentVelocity.z))
            {
                Debug.LogWarning("currentVelocity is NaN, resetting to zero");
                currentVelocity = Vector3.zero;
            }

            // Apply movement
            if (walking && isOnGround)
            {
                rb.linearVelocity = new Vector3(currentVelocity.x, rb.linearVelocity.y, currentVelocity.z);
            }
            else
            {
                rb.linearVelocity = currentVelocity;
            }
            // Handle rotation with turn speed limit
            if (desiredDir != Vector3.zero && desiredDir.magnitude > 0.001f)
            {
                Vector3 newForward;
                if (angle > maxTurnThisFrame && angle > 0.001f)
                {
                    // Limit rotation to turnSpeed - turn as much as possible this frame
                    newForward = Vector3.Slerp(lastForward, desiredDir, maxTurnThisFrame / angle);
                }
                else
                {
                    // Can complete the turn this frame
                    newForward = desiredDir;
                }
                if (controlRotation) { 
                    // Validate newForward
                    if (!float.IsNaN(newForward.x) && !float.IsNaN(newForward.y) && !float.IsNaN(newForward.z) && newForward.magnitude > 0.001f)
                    {
                        Quaternion targetRotation = Quaternion.LookRotation(newForward, Vector3.up);

                        if (stayUpright)
                        {
                            // Keep rotation upright by locking X and Z axes
                            Vector3 euler = targetRotation.eulerAngles;
                            euler.x =    0f;
                            euler.z = 0f;
                            targetRotation = Quaternion.Euler(euler);
                        }

                        rb.MoveRotation(targetRotation);
                        lastForward = newForward;
                    }
                }
            }
        }

        private float PathDistance()
        {
            if (currentPath == null || currentPath.length == 0 || currentWaypoint >= currentPath.length)
                return 0;

            Vector3 currentPos = transform.position;
            return currentPath.ClosestDistanceToPoint(currentPos);
        }
        private int FindBestNextWaypoint()
        {
            /*
             the best next waypoint probably should account for speed
             */
            
           float minDistance = float.MaxValue;
           int bestIndex = -1;

           for (int i = currentWaypoint-1; i < currentPath.length; i++)
           {
               float dist = Vector3.Distance(transform.position, currentPath[i]);
               if (dist < minDistance)
               {
                   minDistance = dist;
                   bestIndex = i;
               }
           }

            return bestIndex+1;
       }

       private void GroundCheck()
       {
           isOnGround = Physics.Raycast(transform.position, Vector3.down, groundCheckDistance);
       }

       public void ApplyStoppingForce()
       {

           if (walking && isOnGround)
           {
               rb.linearVelocity = new Vector3(
                   Mathf.Lerp(rb.linearVelocity.x, 0f, deceleration * Time.fixedDeltaTime),
                   rb.linearVelocity.y,
                   Mathf.Lerp(rb.linearVelocity.z, 0f, deceleration * Time.fixedDeltaTime)
               );
           }
           else
           {
                
                rb.linearVelocity = Vector3.Lerp(rb.linearVelocity, Vector3.zero, deceleration * Time.fixedDeltaTime);
           }
       }

       public int GetAStarPathLength() => currentAStarPath?.Count ?? 0;

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
           reachedDestination = false;
           currentNode = OctNavigation.GetGraph(walking).GetClosestNode(transform.position);
           if (currentGoal.HasValue)
           {
                currentWaypoint = 0;
                PathToPoint(currentGoal.Value);
                targetCachedPosition = currentGoal.Value;
           }
           else
           {
               Debug.LogAssertion("Target has not been assigned to the agent: " + gameObject.name);
           }
        }
        public void SetDestination(Vector3 point)
        {
            straightPath = false;
            manualDestination = point;
            target = null;
            isPaused = false;
            isMoving = true;
            ForceRepath();
        }

        public void SetTarget(Transform newTarget = null)
        {
            straightPath = false;
            if (newTarget != null) target = newTarget;
            forceNewPath = false;
            Graph currentGraph = OctNavigation.GetGraph(walking);
            if (target == null || currentGraph == null || currentGraph.nodes.Count == 0)
            {
                Debug.LogError("Target or graph is null.");
                Stop();
                return;
            }
            manualDestination = null;
            isPaused = false;
            isMoving = true;
            ForceRepath();
        }
        public void SetDestinationStraight(Vector3? point = null)
        {
          
            Vector3? dest = point ?? currentGoal ?? null;
            if (!dest.HasValue) return;
            straightPath = true;
            manualDestination = dest;
            target = null;
            isPaused = false;
            isMoving = true;
            ForceRepath();
        }

        public void PathToPoint(Vector3 goalPosition)
        {
            forceNewPath = false;
            if (straightPath)
            {
                currentPath = BuildStraightPath(goalPosition);
                return;
            }

            Graph currentGraph = OctNavigation.GetGraph(walking);
            if (currentGraph == null || currentGraph.nodes.Count == 0)
            {
                Debug.LogError("graph is null.");
                return;
            }

            targetCachedPosition = goalPosition;

            currentNode = currentGraph.GetClosestNode(transform.position);
            OctNode targetNode = currentGraph.GetClosestNode(currentGoal.Value);

            if (currentNode == targetNode)
            {
                currentAStarPath = new List<Node> { currentGraph.FindNode(currentNode) };
                currentPath.Clear();
                currentPath.Add(goalPosition);
                currentWaypoint = 0;
                return;
            }
            float startTime = Time.realtimeSinceStartup;
            currentAStarPath = currentGraph.AStar(currentNode, targetNode, heuristicType);
            float endTime = Time.realtimeSinceStartup;
            if (walking)
            {
                List<Node> pathToUse = currentAStarPath;
                // first node is always fine 
                List<Node> truncated = new List<Node> { currentAStarPath[0] };
                float lastY = currentAStarPath[0].center.y;
                for (int i = 1; i < currentAStarPath.Count; i++)
                {
                    float thisY = currentAStarPath[i].center.y;

                    if (thisY - lastY > maxStepDistance)
                    {
                        break;
                    }
                    truncated.Add(currentAStarPath[i]);
                    lastY = thisY;
                }
                pathToUse = truncated;
                currentAStarPath = pathToUse;
            }
            if (currentAStarPath == null || currentAStarPath.Count == 0)
            {
                Debug.LogWarning("No path found in PathToTarget");
                isPreloadingNextPath = false;
                forceNewPath = true;
                return;
            }

            bool reachedTarget = currentAStarPath.Last().octreeNode == targetNode;
            Vector3 finalPoint = reachedTarget ? goalPosition : currentAStarPath.Last().center;

            currentWaypoint = 0;
            if (enableSmoothing)
            {
                if (enableRaycastSmoothing)
                {
                    
                    currentPath = BuildSmoothedPathPhysics(currentAStarPath, finalPoint);
                }
                else
                {
                    currentPath = BuildSmoothedPath(currentAStarPath, finalPoint);

                }
            }
            else
            {
                currentPath = new AgentPath();
                foreach (Node node in currentAStarPath)
                {
                    currentPath.Add(node.center);
                }
            }
        }
        public void Stop()
        {
            isMoving = false;
            isPaused = false;
            straightPath = false;
            ClearPath();
            ZeroVelocity();
            
        }
        public void Pause()
        {
            if (!isMoving) return;
            isPaused = true;
            cashedVelocity = currentVelocity;
            cashedAngularVelocity = rb.angularVelocity;
            rotationFrozen = true;
            ZeroVelocity();
        }

        public void Resume()
        {
            if (!isPaused) return;
            isPaused = false;
            currentVelocity = cashedVelocity;
            rb.angularVelocity = cashedAngularVelocity;
            rotationFrozen = false;
            if (rb != null) rb.linearVelocity = cashedVelocity;
        }
        public class PathSubscription : IDisposable
        {
            readonly OctreeNavigationAgent youtuberAgent;
            readonly Action notificationBell;
            public event Action OnReached;

            internal PathSubscription(OctreeNavigationAgent agent)
            {
                youtuberAgent = agent;
                notificationBell = () =>
                {
                    OnReached?.Invoke();
                };
                youtuberAgent.OnDestinationReached += notificationBell;
            }

            public void Dispose()
            {
                youtuberAgent.OnDestinationReached -= notificationBell;
            }
        }
        public PathSubscription BeginMovement()
        {
            if (!currentGoal.HasValue)
            {
                Debug.LogWarning("Nno target or destination set");
                return null;
            }
            rotationFrozen = false;
            isPaused = false;
            isMoving = true;
            ForceRepath();
            return new PathSubscription(this);
        }
        private void ClearPath()
        {
            currentPath.Clear();
            currentAStarPath.Clear();
            viaPoints.Clear();
            portals.Clear();
            currentWaypoint = 0;
        }
        private void ZeroVelocity()
        {
            currentVelocity = Vector3.zero;
            if (rb != null)
            {
                rb.linearVelocity = Vector3.zero;
                rb.angularVelocity = Vector3.zero;
            }
        }
     
        private void ForceRepath()
        {
            ClearPath();
            forceNewPath = true;
            isPreloadingNextPath = false;
            GetNewPath();
            OnPathUpdated?.Invoke(currentPath.waypoints);
        }
     
        public void ResetPath()
        {
            currentPath = new AgentPath();
            currentAStarPath =  new List<Node>();
        }
        public AgentPath BuildSmoothedPath(List<Node> rawPath, Vector3 finalTarget, float? circleRadius = null)
        {
            portals = ExtractPortals(rawPath);

            viaPoints = new List<Vector3>() ;
            for(int i = 0; i < portals.Count; i++)
            {
                List<Vector3> portal = portals[i];

                Vector3 from = (i == 0) ? transform.position : currentAStarPath[i].center;
                Vector3 to = (i == portals.Count - 1) ? finalTarget : currentAStarPath[i + 1].center;

                Vector3 normal = Vector3.Cross(portal[1] - portal[0], portal[2] - portal[1]).normalized;
                Plane portalPlane = new Plane(normal, portal[0]);

                Vector3 travelDir = (to - from).normalized;
                Vector3 intersection;

                if (portalPlane.Raycast(new Ray(from, travelDir), out float hitDist))
                {
                    intersection = from + travelDir * hitDist;
                }
                else
                {
                    intersection = portal.Aggregate(Vector3.zero, (acc, p) => acc + p) / portal.Count;
                }

                Bounds portalBounds = new Bounds(portal[0], Vector3.zero);
                foreach (Vector3 p in portal) portalBounds.Encapsulate(p);
                Vector3 clamped = portalBounds.ClosestPoint(intersection);

                viaPoints.Add(clamped);
            }

            viaPoints.Add(finalTarget);
            AgentPath path = SplineThrough(viaPoints, splineSubdivisionCount);

            if (path.length > 1 && Vector3.Distance(path[0], transform.position) < Mathf.Epsilon)
            {
                path.waypoints.RemoveAt(0);
            }

            if (walking)
            {
                for (int i = 0; i < path.length; i++)
                {
                    float t = (path.length == 1) ? 0f : (float)i / (path.length - 1);
                    int rawIndex = Mathf.Clamp(Mathf.RoundToInt(t * (rawPath.Count - 1)), 0, rawPath.Count - 1);

                    float rawY = rawPath[rawIndex].octreeNode.bounds.center.y + rawPath[rawIndex].octreeNode.bounds.extents.y;

                    path.waypoints[i] = new Vector3(path.waypoints[i].x, rawY, path.waypoints[i].z);
                }
            }

            return path;
        }
        public AgentPath BuildSmoothedPathPhysics(List<Node> rawPath, Vector3 finalTarget, int subdivs = 8)
        {
            if (walking)
            {
                return BuildSmoothedPath(rawPath, finalTarget);
            }
            List<Vector3> points = new List<Vector3>() { transform.position };
            AgentPath path = new AgentPath();
            points.AddRange(rawPath.Select(n => n.center).ToList());
            points.Add(finalTarget);

            if(points.Count < 2)
            {
                foreach(Vector3 p in points)
                {
                    path.Add(p);
                }
                return path;
            }

            viaPoints = new List<Vector3>() { points[0] };
            int i = 0;
            while (i < points.Count - 1)
            {
                int best = i +1;

                for (int j = points.Count - 1; j > best; j--)
                {
                    float raySize = Vector3.Distance(points[i], points[j]);
                    
                    if (!Physics.SphereCast(points[i], agentRadius , (points[j] - points[i]), out RaycastHit hit, raySize, layerMask))
                    {
                        //Debug.DrawRay(points[i], (points[j] - points[i]).normalized * raySize, OctColour.SpringGreen.Color(), 30f);
                        //OctUtils.DrawBounds2D(new Bounds(points[j], Vector3.one * 5f), OctColour.SpringGreen.Color());
                        best = j;
                        break;
                    }
                    else { 
                        //Debug.DrawRay(points[i], (points[j] - points[i]).normalized * raySize, Color.red, 30f);
                        //OctUtils.DrawBounds2D(new Bounds(points[j], Vector3.one * 5f), OctColour.Crimson.Color());
                    }
                }
                viaPoints.Add(points[best]);
                i = best;

            }
           
            for (int k = 0; k < viaPoints.Count-1; k++)
            {
                Vector3 p0 = (k == 0) ? viaPoints[0] : viaPoints[k - 1];
                Vector3 p1 = viaPoints[k];
                Vector3 p2 = viaPoints[k + 1];
                Vector3 p3 = (k+2 < viaPoints.Count) ? viaPoints[k + 2] : viaPoints[^1];

                for(int step = 0; step < subdivs; step++)
                {
                    float t = step / (float)subdivs;
                    Vector3 point = CatmullRom(p0, p1, p2, p3, t);
                    path.Add(point);
                }
            }
            path.Add(viaPoints[^1]);
            if (path.length > 1 && Vector3.Distance(path[0], transform.position) < Mathf.Epsilon)
            {
                path.waypoints.RemoveAt(0);
            }

            return path;

        }
        public AgentPath BuildStraightPath(Vector3? target = null)
        {
            AgentPath path = new AgentPath();
            Vector3 start = transform.position;
            path.Add(start);
            Vector3? end = target ?? currentGoal ?? null;
            if(!end.HasValue)
            {
                // no path will just have one node
                return path;
            }
            if (Physics.Raycast(start, (end.Value - start).normalized, out RaycastHit hit, Vector3.Distance(start, end.Value),layerMask))
            {
                path.Add(hit.point);
            }
            else
            { 
                path.Add(end.Value);
            }
            
            return path;
        }

        private void PreloadAndMergePath()
        {
            Debug.Log("dynamic repath");
            Graph graph = OctNavigation.GetGraph(walking);
            OctNode startOct = graph.GetClosestNode(transform.position);
            OctNode endOct = graph.GetClosestNode(nextGoal.Value);
            List<Node> newRaw = graph.AStar(startOct, endOct, heuristicType);
            if (newRaw == null || newRaw.Count == 0)
            {
                forceNewPath = true;
                isPreloadingNextPath = false;
                return;
            }

            List<Node> remainingOldPath = currentAStarPath.Skip(Mathf.Clamp(GetCurrentRawIndex(), 0, currentAStarPath.Count - 1)).ToList();

            if (remainingOldPath.Count > 0 && remainingOldPath.Last().Equals(newRaw[0]))
            {
                newRaw.RemoveAt(0);
            }

            currentAStarPath = remainingOldPath.Concat(newRaw).ToList();


            Vector3 finalPoint = (newRaw.Last().octreeNode == endOct)
                                 ? currentGoal.Value
                                 : newRaw.Last().center;

            if (enableSmoothing)
            {
                if (enableRaycastSmoothing)
                {

                    currentPath = BuildSmoothedPathPhysics(currentAStarPath, finalPoint);
                }
                else
                {
                    currentPath = BuildSmoothedPath(currentAStarPath, finalPoint);

                }
            }
            else
            {
                currentPath = new AgentPath();
                foreach (Node node in currentAStarPath)
                {
                    currentPath.Add(node.center);
                }
            }
            nextGoal = null;
            currentWaypoint = 0;
            OnPathUpdated?.Invoke(currentPath.waypoints);
            isPreloadingNextPath = false;
        }
        private int GetCurrentRawIndex()
        {
            if (currentAStarPath == null || currentAStarPath.Count == 0) return 0;

            Vector3 probe = (currentWaypoint < currentPath.length) ? currentPath[currentWaypoint] : transform.position;

            float bestDist = float.MaxValue;
            int bestIndex = 0;

            for (int i = 0; i < currentAStarPath.Count; i++)
            {
                // scuffed but should be close enough
                float dist = Vector3.Distance(probe, currentAStarPath[i].center);
                if (dist < bestDist)
                {
                    bestDist = dist;
                    bestIndex = i;
                }
            }

            return bestIndex;
        }
        private AgentPath SplineThrough(List<Vector3> pts, int subdivs = 8, Vector3? circleCenter = null)
        {
           AgentPath path = new AgentPath();
           if (pts.Count < 2)
           {
               foreach (Vector3 p in pts) path.Add(p);
               return path;
           }

           // get ghost points
           Vector3 Get(int i)
           {
               if (i < 0) return pts[0];
               else if (i >= pts.Count) return pts[^1];
               else return pts[i];
           }

           // build segments
           for (int i = 0; i < pts.Count - 1; i++)
           {
               Vector3 p0 = Get(i - 1);
               Vector3 p1 = Get(i);
               Vector3 p2 = Get(i + 1);
               Vector3 p3 = Get(i + 2);

/*              if (i == pts.Count - 2 && circleCenter.HasValue)
                {
                    p3 = ComputeGhostPoint(p1, p2, circleCenter.Value);
                }
                else
                {
                   
                }*/

                for (int step = 0; step < subdivs; step++)
               {
                   float t = step / (float)subdivs;
                   Vector3 point = CatmullRom(p0,p1,p2,p3,t);
                   path.Add(point);
               }
           }
           path.Add(pts[^1]);
           return path;
       }

        //circle around target instead of going straight to it
        private Vector3 ComputeGhostPoint(Vector3 prev, Vector3 last, Vector3 center) 
        {
            float delta = Vector3.Distance(last, prev);

            Vector3 R = last - center;
            R.y = 0;

            R.Normalize();

            Vector3 T = Vector3.Cross(Vector3.up, R).normalized;

            return last + T * delta;
        }

        public static Vector3 CatmullRom(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t)
        {
            return 0.5f * (
                2f * p1 +
                (-p0 + p2) * t +
                (2f * p0 - 5f * p1 + 4f * p2 - p3) * t * t +
                (-p0 + 3f * p1 - 3f * p2 + p3) * t * t * t
            );
        }

        public List<List<Vector3>> ExtractPortals(List<Node> path)
       {
           List<List<Vector3>> portals = new();

           for (int i = 0; i < path.Count - 1; i++)
           {
               Bounds a = path[i].bounds;
               Bounds b = path[i + 1].bounds;
               List<Vector3> portal = GetTouchingPortal(a, b);

               if (portal != null && portal.Count == 4)
               {
                   portals.Add(portal); 
               }
              /* else
                   Debug.LogWarning($"Could not compute portal between node {i} and {i + 1}");*/
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

        private void OnDrawGizmosSelected()
        {
            Gizmos.color = OctColour.Chartreuse.Color();
            Gizmos.DrawWireSphere(transform.position, accuracy);
            if (OctNavigation.GetGraph(walking) != null && GetAStarPathLength() > 0){

            

                Gizmos.color = Color.blue;
                for (int i = 0; i < GetAStarPathLength(); i++)
                {
                    Gizmos.DrawWireSphere(GetPathNode(i).bounds.center, 0.5f);
                    if (i < GetAStarPathLength() - 1)
                    {   
                        Gizmos.DrawLine(GetPathNode(i).bounds.center, GetPathNode(i + 1).bounds.center);
                    }
                }
                Gizmos.color = OctColour.Teal.Color();
                Gizmos.DrawWireSphere(GetPathNode(0).bounds.center, 2f);

                Gizmos.color = Color.red;
                Gizmos.DrawWireSphere(GetPathNode(GetAStarPathLength() - 1).bounds.center, 2f);
            }
            if (currentGoal != null)
            {
                Gizmos.color = Color.magenta;
                Gizmos.DrawLine(transform.position, currentGoal.Value);
            }
            if (viaPoints != null)
            {
                Gizmos.color = Color.cyan;
                foreach (Vector3 p in viaPoints)
                {
                    Gizmos.DrawSphere(p, 0.7f);
                }
            }

            if (currentPath.length > 0)
            {
               
                for (int i = 0; i < currentPath.length; i++)
                {
                    if(i == currentPath.length - 1)
                    {
                        Gizmos.color = Color.red;
                    }
                    else if (i == currentWaypoint)
                    {
                        Gizmos.color = Color.green;
                    }
                    else
                    {
                        Gizmos.color = Color.white;
                    }
                    Vector3 wp = currentPath[i];
                    Gizmos.DrawSphere(wp, 0.15f);

                    if (i < currentPath.length - 1)
                    {
                        Vector3 nextWp = currentPath[i + 1];
                        Gizmos.DrawLine(wp, nextWp);
                    }
                }
            }
            if (OctNavigation.GetGraph(walking) == null) return;
            foreach (List<Vector3> p in portals)
            {
                for (int i = 0; i < 4; i++)
                {
                    Gizmos.color = Color.magenta;
                    Gizmos.DrawLine(p[i], p[(i + 1) % 4]);
                }
            }
        }

 
    }
}