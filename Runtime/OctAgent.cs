using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using OctNav;

namespace OctNav
{
    [RequireComponent(typeof(Rigidbody))]
    public class OctAgent : MonoBehaviour
    {
        [Header("Base Locomotion")]
        public float maxSpeed = 5f;
        public float acceleration = 5f;
        public float accuracy = 1f;
        public float turnSpeed = 5f;
        public float stoppingDistance = 5f;
        public float deceleration = 5f;

        [Header("Acceleration Profile")]
        [Tooltip("Relates acceleration multiplier (y) to delta velocity (x)")]
        public AnimationCurve accelerationGraph = AnimationCurve.EaseInOut(0f, 0.5f, 1f, 3f);

        [Header("Turning Speed Curve")]
        [Tooltip("Relates velocity multiplier (y) to turning degrees per second (x)")]
        public AnimationCurve turnRateToSpeed = AnimationCurve.Linear(0f, 1f, 100f, 0.3f);

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

        [Header("Velocity-Dependent Turning")]
        public float minTurnSpeedBoostVelocity = 0.5f; // Velocity below which boost starts
        public float maxTurnSpeedBoostFactor = 2.0f; // Max multiplier for turnSpeed when very slow

        [Header("Aggressive Turning for Large Angles")]
        public float snapTurnAngleThreshold = 60f; // Angle (degrees) beyond which an aggressive turn boost is applied
        public float snapTurnAdditionalFactor = 2.0f; // Additional multiplier for turn speed when angle is very large

        [Header("Octree Information")]
        public Transform target;

        [Header("Pathfinding Settings")]
        public HeuristicType heuristicType = HeuristicType.Euclidean;
        public bool walking = false;
        public bool straightPath = false;
        public bool stayUpright = true;
        public bool controlRotation = true;
        public float maxStepDistance = 1f;

        public AgentPath currentPath = new AgentPath();
        private List<OctNavGraph.GraphNode> currentAStarPath = new List<OctNavGraph.GraphNode>();

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

                currentNode = OctManager.GetGraph(walking).GetClosestNode(transform.position);
            lastForward = transform.forward;
            isPreloadingNextPath = false;
            forceNewPath = true;
        }

        private void FixedUpdate()
        {
            // Pre-checks: Ensure necessary components and goals are present
            if (OctManager.GetGraph(walking) == null || currentGoal == null) return;

            GroundCheck();

            // Check for target position deviation to force a new path
            if (Vector3.Distance(targetCachedPosition, currentGoal.Value) > targetMoveThreshold && !straightPath)
            {
                isPreloadingNextPath = false;
                forceNewPath = true;
            }

            // Handle destination reached or pausing
            if (!reachedDestination && !isMoving)
            {
                reachedDestination = true;
                ApplyStoppingForce();
                OnDestinationReached?.Invoke();
            }
            if (!isMoving || isPaused)
            {
                //ApplyStoppingForce();
                return;
            }

            /* if (stayUpright)
             {
                 Quaternion uprightRotation = Quaternion.LookRotation(transform.forward, Vector3.up);
                 rb.MoveRotation(uprightRotation);

             }*/

            // Check distance to final goal for stopping
            float distToGoal = Vector3.Distance(transform.position, currentGoal.Value);
            if (distToGoal < stoppingDistance)
            {
                reachedDestination = true;
                ApplyStoppingForce();
                isMoving = false;
                OnDestinationReached?.Invoke();
                return;
            }

            // Waypoint progression
            Vector3 nextWaypointPosition = currentPath[currentWaypoint];
            float distToNextWaypoint = Vector3.Distance(transform.position, nextWaypointPosition);

            if (distToNextWaypoint < accuracy)
            {
                currentWaypoint++;
                                                             // Clamp currentWaypoint to prevent out-of-bounds access
                if (currentWaypoint >= currentPath.Length)
                {
                    currentWaypoint = currentPath.Length - 1;
                }
            }

            // Path re-calculation and recovery
            if (!straightPath)
            {
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

            // Handle empty or invalid path
            if (currentPath == null || currentPath.Length == 0 || currentWaypoint < 0 || currentWaypoint >= currentPath.Length)
            {
                forceNewPath = true;
                isPreloadingNextPath = false;
                isMoving = false;
                reachedDestination = true;
                Debug.LogWarning("Path is invalid or empty, forcing new path calculation.");
                return;
            }

            // Dynamic Repathing
            if (enableDynamicRepathing && nextGoal.HasValue)
            {
                int remaining = currentAStarPath.Count - 1 - GetCurrentRawIndex();
                if (!isPreloadingNextPath && remaining < nodesAwayBeforeRepath && !straightPath && isMoving && !forceNewPath)
                {
                    PreloadAndMergePath();
                    isPreloadingNextPath = true;
                }
            }

            // Core Movement and Rotation Logic
            Vector3 desiredDir = (nextWaypointPosition - transform.position).normalized;

            // Ensure lastForward is always valid, especially at start
            if (lastForward.magnitude < 0.001f)
            {
                lastForward = transform.forward;
            }

            // Calculate the angle to the desired direction
            float angleToDesired = Vector3.Angle(lastForward, desiredDir);

            // Dynamic Turn Speed Adjustment based on Current Velocity
            // Calculate a boost factor for turn speed. Higher when current velocity is low
            float currentSpeedNormalized = Mathf.Clamp01(currentVelocity.magnitude / minTurnSpeedBoostVelocity);
            float turnSpeedBoost = Mathf.Lerp(maxTurnSpeedBoostFactor, 1.0f, currentSpeedNormalized); // Lerp from maxBoost (slow) to 1.0 (fast)

            float effectiveTurnSpeed = turnSpeed * turnSpeedBoost;

            // Aggressive Turn Boost for Large Angles (to prevent spinning)
            // If the angle to desired direction is very large, allow for an even sharper turn
            if (angleToDesired > snapTurnAngleThreshold)
            {
                effectiveTurnSpeed *= snapTurnAdditionalFactor;
                Debug.Log($"[Turn Debug] Applying AGGRESSIVE Turn Boost! Angle: {angleToDesired:F2} deg. New Effective Turn Speed: {effectiveTurnSpeed:F2}");
            }


            // Calculate the maximum angle the agent can turn in this FixedUpdate frame with ALL boosts
            float maxTurnThisFrame = effectiveTurnSpeed * Time.fixedDeltaTime;

            // Determine the actual angle the agent will turn this frame
            // This clamps the turn to the agent's max effective turn speed if the desired turn is too sharp
            float actualTurnAngleThisFrame = Mathf.Min(angleToDesired, maxTurnThisFrame);

            // Calculate the ACTUAL turn rate in degrees per second
            float turnRateDegPerSec = actualTurnAngleThisFrame * (1  / Time.deltaTime);

            // Debugging for turn calculation
            Debug.Log($"[Turn Debug] Current Velocity Magnitude: {currentVelocity.magnitude:F2}");
            Debug.Log($"[Turn Debug] Turn Speed Boost Factor (Vel): {turnSpeedBoost:F2}");
            Debug.Log($"[Turn Debug] Effective Turn Speed (Pre-Aggro): {turnSpeed * turnSpeedBoost:F2}");
            Debug.Log($"[Turn Debug] Effective Turn Speed (Post-Aggro): {effectiveTurnSpeed:F2}");
            Debug.Log($"[Turn Debug] Angle to Desired: {angleToDesired:F2} deg");
            Debug.Log($"[Turn Debug] Max Turn This Frame: {maxTurnThisFrame:F2} deg");
            Debug.Log($"[Turn Debug] Actual Turn Angle This Frame: {actualTurnAngleThisFrame:F2} deg");
            Debug.Log($"[Turn Debug] Turn Rate (Deg/Sec): {turnRateDegPerSec:F2}");

            // Sample the AnimationCurve to get the speed multiplier
            // This curve still dictates how velocity is affected by the actual turning rate.
            float turnSpeedMultiplier = Mathf.Clamp01(turnRateToSpeed.Evaluate(turnRateDegPerSec));
            Debug.Log($"[Turn Debug] Turn Speed Multiplier: {turnSpeedMultiplier:F2}");


            // Handle slopes if walking and on ground
            if (walking && isOnGround)
            {
                RaycastHit hit;
                if (Physics.Raycast(transform.position, Vector3.down, out hit, groundCheckDistance * 2f))
                {
                    float slopeAngle = Vector3.Angle(hit.normal, Vector3.up);
                    if (slopeAngle > maxSlopeAngle)
                    {
                        Vector3 projectedDir = Vector3.ProjectOnPlane(desiredDir, hit.normal);
                        if (projectedDir.magnitude > 0.001f)
                        {
                            desiredDir = projectedDir.normalized;
                        }
                        else
                        {
                            ApplyStoppingForce();
                            Debug.LogWarning("Agent stopped due to extreme slope projection.");
                            return;
                        }
                    }
                }
            }

            // Determine the actual movement direction considering turn limitations
            Vector3 movementDir;
            if (angleToDesired > maxTurnThisFrame && angleToDesired > 0.001f)
            {
                movementDir = Vector3.Slerp(lastForward, desiredDir, actualTurnAngleThisFrame / angleToDesired);
            }
            else
            {
                movementDir = desiredDir;
            }

            // Validate and normalize movementDir
            if (float.IsNaN(movementDir.x) || float.IsNaN(movementDir.y) || float.IsNaN(movementDir.z))
            {
                Debug.LogWarning("movementDir resulted in NaN, using desiredDir as fallback.");
                movementDir = desiredDir;
            }
            if (movementDir.magnitude < 0.001f)
            {
                Debug.LogWarning("movementDir is near zero, setting to transform.forward.");
                movementDir = transform.forward;
            }
            movementDir = movementDir.normalized;


            // Calculate target velocity applying the turn speed multiplier
            Vector3 targetVelocity = movementDir * maxSpeed * turnSpeedMultiplier;

            // Validate targetVelocity
            if (float.IsNaN(targetVelocity.x) || float.IsNaN(targetVelocity.y) || float.IsNaN(targetVelocity.z))
            {
                Debug.LogWarning("targetVelocity is NaN, stopping movement and returning.");
                currentVelocity = Vector3.zero;
                return;
            }

            isMoving = true;

            // Apply acceleration/deceleration
            Vector3 difference = targetVelocity - currentVelocity;
            float deltaSpeed = difference.magnitude;
            float normalizedDelta = Mathf.Clamp01(deltaSpeed / maxSpeed); // Normalize difference

            // Evaluate acceleration curve and apply
            float accelFactor = accelerationGraph.Evaluate(normalizedDelta);
            currentVelocity += difference.normalized * accelFactor * acceleration * Time.fixedDeltaTime;

            // Final validation of currentVelocity
            if (float.IsNaN(currentVelocity.x) || float.IsNaN(currentVelocity.y) || float.IsNaN(currentVelocity.z))
            {
                Debug.LogWarning("currentVelocity is NaN, resetting to zero.");
                currentVelocity = Vector3.zero;
            }

            // Apply linear velocity to Rigidbody
            if (walking && isOnGround)
            {
                rb.linearVelocity = new Vector3(currentVelocity.x, rb.linearVelocity.y, currentVelocity.z);
            }
            else
            {
                rb.linearVelocity = currentVelocity;
            }

            // Handle agent's rotation
            if (controlRotation)
            {
                if (desiredDir != Vector3.zero && desiredDir.magnitude > 0.001f)
                {
                    Vector3 newForwardRotation;
                    if (angleToDesired > maxTurnThisFrame && angleToDesired > 0.001f)
                    {
                        newForwardRotation = Vector3.Slerp(lastForward, desiredDir, maxTurnThisFrame / angleToDesired);
                    }
                    else
                    {
                        newForwardRotation = desiredDir;
                    }

                    // Validate newForwardRotation before applying
                    if (!float.IsNaN(newForwardRotation.x) && !float.IsNaN(newForwardRotation.y) && !float.IsNaN(newForwardRotation.z) && newForwardRotation.magnitude > 0.001f)
                    {
                        Quaternion targetRotation = Quaternion.LookRotation(newForwardRotation, Vector3.up);

                        // Keep rotation upright if stayUpright is enabled
                        if (stayUpright)
                        {
                            Vector3 euler = targetRotation.eulerAngles;
                            euler.x = 0f;
                            euler.z = 0f;
                            targetRotation = Quaternion.Euler(euler);
                        }

                        rb.MoveRotation(targetRotation);
                        lastForward = newForwardRotation.normalized;
                    }
                    else
                    {
                        Debug.LogWarning("newForwardRotation resulted in NaN or zero magnitude, skipping rotation.");
                    }
                }
            }
            else
            {
                lastForward = transform.forward;
            }
        }

        private float PathDistance()
        {
            if (currentPath == null || currentPath.Length == 0 || currentWaypoint >= currentPath.Length)
                return 0;

            Vector3 currentPos = transform.position;
            return currentPath.ClosestDistanceToPoint(currentPos);
        }

        private int FindBestNextWaypoint()
        {
            
           float minDistance = float.MaxValue;
           int bestIndex = -1;

           for (int i = currentWaypoint-1; i < currentPath.Length; i++)
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

        // Slows the agent down
        private void ApplyStoppingForce()
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

        /// <summary>
        /// Gets the number of nodes in the current A* path.
        /// </summary>
        /// <returns>The length of the current A* path. Returns 0 if the path is null.</returns>
        public int GetAStarPathLength() => currentAStarPath?.Count ?? 0;

        /// <summary>
        /// Retrieves the OctNode at the specified index in the current A* path.
        /// </summary>
        /// <param name="index">The index of the node to retrieve.</param>
        /// <returns>
        /// The <see cref="OctNode"/> at the given index, or null if the index is out of bounds or the path is invalid.
        /// </returns>
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
            currentNode = OctManager.GetGraph(walking).GetClosestNode(transform.position);
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

        /// <summary>
        /// Sets a manual destination point for movement, resetting any target and forcing a repath.
        /// </summary>
        /// <param name="point">The target destination as a Vector3.</param>
        public void SetDestination(Vector3 point)
        {
            straightPath = false;
            manualDestination = point;
            target = null;
            isPaused = false;
            isMoving = true;
            ForceRepath();
        }
        /// <summary>
        /// Sets a new movement target using a Transform. Resets destination and forces a new path if the graph is valid.
        /// </summary>
        /// <param name="newTarget">The target Transform to follow. Can be null to clear the target.</param>
        public void SetTarget(Transform newTarget = null)
        {
            straightPath = false;
            if (newTarget != null) target = newTarget;
            forceNewPath = false;
            OctNavGraph currentGraph = OctManager.GetGraph(walking);
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

        /// <summary>
        /// Sets a straight-line movement destination. Defaults to current goal if point is not provided.
        /// </summary>
        /// <param name="point">Optional destination point. If null, uses the current goal if available.</param>
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

        /// <summary>
        /// Computes a path to the given goal position using A* or straight path logic.
        /// </summary>
        /// <param name="goalPosition">The position to navigate toward.</param>
        public void PathToPoint(Vector3 goalPosition)
        {
            forceNewPath = false;
            if (straightPath)
            {
                currentPath = BuildStraightPath(goalPosition);
                return;
            }

            OctNavGraph currentGraph = OctManager.GetGraph(walking);
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
                currentAStarPath = new List<OctNavGraph.GraphNode> { currentGraph.FindNode(currentNode) };
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
                List<OctNavGraph.GraphNode> pathToUse = currentAStarPath;
                // first node is always fine 
                List<OctNavGraph.GraphNode> truncated = new List<OctNavGraph.GraphNode> { currentAStarPath[0] };
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
                foreach (OctNavGraph.GraphNode node in currentAStarPath)
                {
                    currentPath.Add(node.center);
                }
            }
        }

        /// <summary>
        /// Stops the agent's movement and optionally resets its velocity.
        /// </summary>
        /// <param name="fullStop">If true, zeroes the velocity completely.</param>
        public void Stop(bool fullStop = false)
        {
            isMoving = false;
            isPaused = false;
            straightPath = false;
            ClearPath();
            if (fullStop)
            { 
                ZeroVelocity();
            }

        }

        /// <summary>
        /// Pauses movement and stores current velocity and angular velocity.
        /// </summary>
        public void Pause()
        {
            if (!isMoving) return;
            isPaused = true;
            cashedVelocity = currentVelocity;
            cashedAngularVelocity = rb.angularVelocity;
            rotationFrozen = true;
            ZeroVelocity();
        }

        /// <summary>
        /// Resumes movement using previously cached velocity and angular velocity.
        /// </summary>
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
            readonly OctAgent eventAgent;
            readonly Action destitantionReachedAction;
            public event Action OnReached;

            internal PathSubscription(OctAgent agent)
            {
                eventAgent = agent;
                destitantionReachedAction = () =>
                {
                    OnReached?.Invoke();
                };
                eventAgent.OnDestinationReached += destitantionReachedAction;
            }

            public void Dispose()
            {
                eventAgent.OnDestinationReached -= destitantionReachedAction;
            }
        }

        /// <summary>
        /// Begins movement toward the current goal. Returns a path subscription for external tracking.
        /// </summary>
        /// <returns>A <see cref="PathSubscription"/> object if goal is valid; otherwise, null.</returns>
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

        /// <summary>
        /// Clears all current pathfinding data including raw and smoothed paths,
        /// portals, and resets the waypoint index.
        /// </summary>
        private void ClearPath()
        {
            currentPath.Clear();
            currentAStarPath.Clear();
            viaPoints.Clear();
            portals.Clear();
            currentWaypoint = 0;
        }

        /// <summary>
        /// Resets the agent's movement velocity to zero, including rigidbody if present.
        /// </summary>
        private void ZeroVelocity()
        {
            currentVelocity = Vector3.zero;
            if (rb != null)
            {
                rb.linearVelocity = Vector3.zero;
                rb.angularVelocity = Vector3.zero;
            }
        }

        /// <summary>
        /// Forces the agent to recalculate a new path immediately from scratch.
        /// Clears old data and triggers path update event.
        /// </summary>
        private void ForceRepath()
        {
            ClearPath();
            forceNewPath = true;
            isPreloadingNextPath = false;
            GetNewPath();
            OnPathUpdated?.Invoke(currentPath.waypoints);
        }

        /// <summary>
        /// Resets the path objects without triggering a full re-path or recalculation.
        /// </summary>
        public void ResetPath()
        {
            currentPath = new AgentPath();
            currentAStarPath = new List<OctNavGraph.GraphNode>();
        }

        /// <summary>
        /// Builds a smoothed movement path using portal-based navigation and Catmull-Rom interpolation.
        /// Applies Y-offset correction if 'walking' is enabled.
        /// </summary>
        public AgentPath BuildSmoothedPath(List<OctNavGraph.GraphNode> rawPath, Vector3 finalTarget, float? circleRadius = null)
        {
            portals = OctUtils.ExtractPortals(rawPath);
            viaPoints = new List<Vector3>();

            for (int i = 0; i < portals.Count; i++)
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

            // Remove redundant first point if coinciding with agent's position
            if (path.Length > 1 && Vector3.Distance(path[0], transform.position) < Mathf.Epsilon)
            {
                path.waypoints.RemoveAt(0);
            }

            // Align Y-position to octree height if walking
            if (walking)
            {
                for (int i = 0; i < path.Length; i++)
                {
                    float t = (path.Length == 1) ? 0f : (float)i / (path.Length - 1);
                    int rawIndex = Mathf.Clamp(Mathf.RoundToInt(t * (rawPath.Count - 1)), 0, rawPath.Count - 1);
                    float rawY = rawPath[rawIndex].octreeNode.bounds.center.y + rawPath[rawIndex].octreeNode.bounds.extents.y;

                    path.waypoints[i] = new Vector3(path.waypoints[i].x, rawY, path.waypoints[i].z);
                }
            }

            return path;
        }

        /// <summary>
        /// Builds a smoothed physics-aware path using raycasting to skip unnecessary waypoints.
        /// Uses Catmull-Rom to interpolate between collision-free segments.
        /// </summary>
        public AgentPath BuildSmoothedPathPhysics(List<OctNavGraph.GraphNode> rawPath, Vector3 finalTarget, int subdivs = 8)
        {
            if (walking)
            {
                return BuildSmoothedPath(rawPath, finalTarget);
            }

            List<Vector3> points = new List<Vector3> { transform.position };
            AgentPath path = new AgentPath();
            points.AddRange(rawPath.Select(n => n.center));
            points.Add(finalTarget);

            if (points.Count < 2)
            {
                foreach (Vector3 p in points)
                {
                    path.Add(p);
                }
                return path;
            }

            viaPoints = new List<Vector3> { points[0] };
            int i = 0;
            while (i < points.Count - 1)
            {
                int best = i + 1;
                for (int j = points.Count - 1; j > best; j--)
                {
                    float raySize = Vector3.Distance(points[i], points[j]);

                    if (!Physics.SphereCast(points[i], agentRadius, (points[j] - points[i]), out _, raySize, layerMask))
                    {
                        best = j;
                        break;
                    }
                }

                viaPoints.Add(points[best]);
                i = best;
            }

            // Interpolate between via points using Catmull-Rom
            for (int k = 0; k < viaPoints.Count - 1; k++)
            {
                Vector3 p0 = (k == 0) ? viaPoints[0] : viaPoints[k - 1];
                Vector3 p1 = viaPoints[k];
                Vector3 p2 = viaPoints[k + 1];
                Vector3 p3 = (k + 2 < viaPoints.Count) ? viaPoints[k + 2] : viaPoints[^1];

                for (int step = 0; step < subdivs; step++)
                {
                    float t = step / (float)subdivs;
                    Vector3 point = OctUtils.CatmullRom(p0, p1, p2, p3, t);
                    path.Add(point);
                }
            }

            path.Add(viaPoints[^1]);

            if (path.Length > 1 && Vector3.Distance(path[0], transform.position) < Mathf.Epsilon)
            {
                path.waypoints.RemoveAt(0);
            }

            return path;
        }

        /// <summary>
        /// Builds a direct straight-line path from the agent to a target point, 
        /// stopping if a collision is detected along the way.
        /// </summary>
        public AgentPath BuildStraightPath(Vector3? target = null)
        {
            AgentPath path = new AgentPath();
            Vector3 start = transform.position;
            path.Add(start);

            Vector3? end = target ?? currentGoal ?? null;
            if (!end.HasValue)
            {
                return path; // Single-node path if no goal
            }

            if (Physics.Raycast(start, (end.Value - start).normalized, out RaycastHit hit, Vector3.Distance(start, end.Value), layerMask))
            {
                path.Add(hit.point);
            }
            else
            {
                path.Add(end.Value);
            }

            return path;
        }

        /// <summary>
        /// Attempts to preload a new A* path to the next goal and smoothly merge it
        /// with the existing path if applicable. Supports optional smoothing modes.
        /// </summary>
        private void PreloadAndMergePath()
        {
            OctNavGraph graph = OctManager.GetGraph(walking);
            OctNode startOct = graph.GetClosestNode(transform.position);
            OctNode endOct = graph.GetClosestNode(nextGoal.Value);
            List<OctNavGraph.GraphNode> newRaw = graph.AStar(startOct, endOct, heuristicType);

            if (newRaw == null || newRaw.Count == 0)
            {
                forceNewPath = true;
                isPreloadingNextPath = false;
                return;
            }

            List<OctNavGraph.GraphNode> remainingOldPath = currentAStarPath.Skip(Mathf.Clamp(GetCurrentRawIndex(), 0, currentAStarPath.Count - 1)).ToList();

            if (remainingOldPath.Count > 0 && remainingOldPath.Last().Equals(newRaw[0]))
            {
                newRaw.RemoveAt(0); // Prevent node duplication
            }

            currentAStarPath = remainingOldPath.Concat(newRaw).ToList();
            Vector3 finalPoint = (newRaw.Last().octreeNode == endOct) ? currentGoal.Value : newRaw.Last().center;

            if (enableSmoothing)
            {
                currentPath = enableRaycastSmoothing
                            ? BuildSmoothedPathPhysics(currentAStarPath, finalPoint)
                            : BuildSmoothedPath(currentAStarPath, finalPoint);
            }
            else
            {
                currentPath = new AgentPath();
                foreach (OctNavGraph.GraphNode node in currentAStarPath)
                {
                    currentPath.Add(node.center);
                }
            }

            nextGoal = null;
            currentWaypoint = 0;
            OnPathUpdated?.Invoke(currentPath.waypoints);
            isPreloadingNextPath = false;
        }

        /// <summary>
        /// Returns the index in the raw A* path closest to the agent's current position or waypoint.
        /// </summary>
        private int GetCurrentRawIndex()
        {
            if (currentAStarPath == null || currentAStarPath.Count == 0) return 0;

            Vector3 probe = (currentWaypoint < currentPath.Length) ? currentPath[currentWaypoint] : transform.position;

            float bestDist = float.MaxValue;
            int bestIndex = 0;

            for (int i = 0; i < currentAStarPath.Count; i++)
            {
                float dist = Vector3.Distance(probe, currentAStarPath[i].center);
                if (dist < bestDist)
                {
                    bestDist = dist;
                    bestIndex = i;
                }
            }

            return bestIndex;
        }

        /// <summary>
        /// Returns the ghost point in the list of points
        /// </summary>
        Vector3 GetGhostPoint(List<Vector3> pts, int i)
        {
            if (i < 0) return pts[0];
            else if (i >= pts.Count) return pts[^1];
            else return pts[i];
        }

        private AgentPath SplineThrough(List<Vector3> pts, int subdivs = 8, Vector3? circleCenter = null)
        {
            AgentPath path = new AgentPath();
            if (pts.Count < 2)
            {
                foreach (Vector3 p in pts) path.Add(p);
                return path;
            }


            // build segments
            for (int i = 0; i < pts.Count - 1; i++)
            {
                 Vector3 p0 = GetGhostPoint(pts, i - 1);
                 Vector3 p1 = GetGhostPoint(pts, i);
                 Vector3 p2 = GetGhostPoint(pts, i + 1);
                 Vector3 p3 = GetGhostPoint(pts, i + 2);
                 /*
                 if (i == pts.Count - 2 && circleCenter.HasValue)
                 {
                     p3 = OctUtils.ComputeGhostPoint(p1, p2, circleCenter.Value);
                 }
                 else
                 {
                    
                 }*/

                 for (int step = 0; step < subdivs; step++)
                 {
                     float t = step / (float)subdivs;
                     Vector3 point = OctUtils.CatmullRom(p0,p1,p2,p3,t);
                     path.Add(point);
                 }
            }
            path.Add(pts[^1]);

           return path;
        }


        private void OnDrawGizmosSelected()
        {
            Gizmos.color = OctColour.Chartreuse.Color();
            Gizmos.DrawWireSphere(transform.position, accuracy);

            if (OctManager.GetGraph(walking) != null && GetAStarPathLength() > 0)
            {
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

            if (currentPath.Length > 0)
            {
               
                for (int i = 0; i < currentPath.Length; i++)
                {
                    if(i == currentPath.Length - 1)
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

                    if (i < currentPath.Length - 1)
                    {
                        Vector3 nextWp = currentPath[i + 1];
                        Gizmos.DrawLine(wp, nextWp);
                    }
                }
            }

            if (OctManager.GetGraph(walking) == null) return;

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