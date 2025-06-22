using System.Collections.Generic;
using UnityEngine;

namespace OctNav
{
    /// <summary>
    /// Represents a navigational path composed of a sequence of waypoints.
    /// Provides utilities for measuring distance, direction, and proximity to points.
    /// </summary>
    public class AgentPath
    {
        /// <summary>
        /// The list of waypoints that define the path.
        /// </summary>
        public List<Vector3> waypoints = new List<Vector3>();

        /// <summary>
        /// Clears all waypoints from the path.
        /// </summary>
        public void Clear()
        {
            waypoints.Clear();
        }

        /// <summary>
        /// Adds a waypoint to the end of the path.
        /// </summary>
        /// <param name="point">The waypoint to add.</param>
        public void Add(Vector3 point)
        {
            waypoints.Add(point);
        }

        /// <summary>
        /// Gets the number of waypoints in the path.
        /// </summary>
        public int Length => waypoints.Count;

        /// <summary>
        /// Gets the total travel distance along the path.
        /// </summary>
        public float travelDistance
        {
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

        /// <summary>
        /// Calculates the shortest distance from a given point to the path.
        /// </summary>
        /// <param name="point">The point to check against the path.</param>
        /// <returns>The shortest distance from the point to any segment of the path.</returns>
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
                t = Mathf.Clamp01(t);
                Vector3 closestPoint = a + t * ab;
                float distance = Vector3.Distance(point, closestPoint);

                if (distance < minDistance)
                {
                    minDistance = distance;
                }
            }

            return minDistance;
        }

        /// <summary>
        /// Gets the normalized direction vector from waypoint i to i+1.
        /// </summary>
        /// <param name="i">The index of the waypoint.</param>
        /// <returns>The direction vector, or Vector3.zero if invalid index.</returns>
        public Vector3 GetDirection(int i)
        {
            if (i < 0 || i >= waypoints.Count - 1 || Length < 2)
            {
                return Vector3.zero;
            }
            Vector3 a = this[i];
            Vector3 b = this[i + 1];
            return (b - a).normalized;
        }

        /// <summary>
        /// Gets the waypoint at the specified index. 
        /// If the index is out of range, returns the last point or Vector3.zero if empty.
        /// </summary>
        /// <param name="i">Index of the waypoint.</param>
        /// <returns>The waypoint at index i.</returns>
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
}
