using System.Collections.Generic;
using UnityEngine;

namespace OctNav {
    public enum OctColour
    {
        VibrantRed = 0,
        VividGreen,
        BrightBlue,
        Yellow,
        Magenta,
        Cyan,
        Orange,
        Lime,
        HotPink,
        Purple,
        Turquoise,
        Chartreuse,
        Crimson,
        Gold,
        SpringGreen,
        DodgerBlue,
        DeepPink,
        Teal,
        Violet,
        Coral
    }

    public static class OctUtils
    {
        private static readonly Color[] colours = new Color[]
        {
            new Color(1f,    0.2f,  0.2f,  1f),   // VibrantRed
            new Color(0.2f,  1f,    0.2f,  1f),   // VividGreen
            new Color(0.2f,  0.2f,  1f,    1f),   // BrightBlue
            new Color(1f,    1f,    0.2f,  1f),   // Yellow
            new Color(1f,    0.2f,  1f,    1f),   // Magenta
            new Color(0.2f,  1f,    1f,    1f),   // Cyan
            new Color(1f,    0.6f,  0.2f,  1f),   // Orange
            new Color(0.6f,  1f,    0.2f,  1f),   // Lime
            new Color(1f,    0.4f,  0.7f,  1f),   // HotPink
            new Color(0.6f,  0.2f,  1f,    1f),   // Purple
            new Color(0.188f,0.835f,0.784f,1f),   // Turquoise   (#30D5C8)
            new Color(0.498f,1f,    0f,    1f),   // Chartreuse  (#7FFF00)
            new Color(0.863f,0.078f,0.235f,1f),   // Crimson     (#DC143C)
            new Color(1f,    0.843f,0f,    1f),   // Gold        (#FFD700)
            new Color(0f,    1f,    0.498f,1f),   // SpringGreen (#00FF7F)
            new Color(0.118f,0.565f,1f,    1f),   // DodgerBlue  (#1E90FF)
            new Color(1f,    0.078f,0.576f,1f),   // DeepPink    (#FF1493)
            new Color(0f,    0.502f,0.502f,1f),   // Teal        (#008080)
            new Color(0.933f,0.510f,0.933f,1f),   // Violet      (#EE82EE)
            new Color(1f,    0.498f,0.314f,1f)    // Coral       (#FF7F50)
        };

        /// <summary>
        /// Gets the Unity Color corresponding to a specific <see cref="OctColour"/> enum value.
        /// </summary>
        /// <param name="colour">The enum value representing a predefined color.</param>
        /// <returns>A Unity <see cref="Color"/> object.</returns>
        public static Color Color(this OctColour colour)
        {
            return colours[(int)colour];
        }

        /// <summary>
        /// Draws a wireframe disk at the specified position using Unity's Gizmos.
        /// </summary>
        /// <param name="position">The center of the disk.</param>
        /// <param name="radius">The radius of the disk.</param>
        /// <param name="color">The color of the wireframe.</param>
        public static void DrawWireDisk(Vector3 position, float radius, Color color)
        {
            Color oldColor = Gizmos.color;
            Gizmos.color = color;
            Matrix4x4 oldMatrix = Gizmos.matrix;
            Gizmos.matrix = Matrix4x4.TRS(position, Quaternion.identity, new Vector3(1, 0.01f, 1));
            Gizmos.DrawWireSphere(Vector3.zero, radius);
            Gizmos.matrix = oldMatrix;
            Gizmos.color = oldColor;
        }

        /// <summary>
        /// Draws a 3D bounding box using Unity's Debug lines.
        /// </summary>
        /// <param name="b">The bounds to draw.</param>
        /// <param name="col">The color of the lines.</param>
        public static void DrawBounds2D(Bounds b, Color col)
        {
            Vector3 c = b.center;
            Vector3 e = b.extents;

            // 8 corners
            Vector3 p0 = c + new Vector3(-e.x, -e.y, -e.z);
            Vector3 p1 = c + new Vector3(e.x, -e.y, -e.z);
            Vector3 p2 = c + new Vector3(e.x, -e.y, e.z);
            Vector3 p3 = c + new Vector3(-e.x, -e.y, e.z);
            Vector3 p4 = c + new Vector3(-e.x, e.y, -e.z);
            Vector3 p5 = c + new Vector3(e.x, e.y, -e.z);
            Vector3 p6 = c + new Vector3(e.x, e.y, e.z);
            Vector3 p7 = c + new Vector3(-e.x, e.y, e.z);

            // bottom face
            Debug.DrawLine(p0, p1, col, 30);
            Debug.DrawLine(p1, p2, col, 30);
            Debug.DrawLine(p2, p3, col, 30);
            Debug.DrawLine(p3, p0, col, 30);

            // top face
            Debug.DrawLine(p4, p5, col, 30);
            Debug.DrawLine(p5, p6, col, 30);
            Debug.DrawLine(p6, p7, col, 30);
            Debug.DrawLine(p7, p4, col, 30);

            // vertical edges
            Debug.DrawLine(p0, p4, col, 30);
            Debug.DrawLine(p1, p5, col, 30);
            Debug.DrawLine(p2, p6, col, 30);
            Debug.DrawLine(p3, p7, col, 30);
        }

        /// <summary>
        /// Calculates a point on a Catmull-Rom spline.
        /// </summary>
        /// <param name="p0">The point before the starting point.</param>
        /// <param name="p1">The starting point.</param>
        /// <param name="p2">The ending point.</param>
        /// <param name="p3">The point after the ending point.</param>
        /// <param name="t">The interpolation parameter (0 to 1).</param>
        /// <returns>The interpolated point on the spline.</returns>
        public static Vector3 CatmullRom(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t)
        {
            return 0.5f * (
                2f * p1 +
                (-p0 + p2) * t +
                (2f * p0 - 5f * p1 + 4f * p2 - p3) * t * t +
                (-p0 + 3f * p1 - 3f * p2 + p3) * t * t * t
            );
        }

        /// <summary>
        /// Computes a virtual "ghost" point used in smoothing or extrapolation.
        /// </summary>
        /// <param name="prev">The previous point in the path.</param>
        /// <param name="last">The last actual point.</param>
        /// <param name="center">The center used for orientation reference.</param>
        /// <returns>The computed ghost point.</returns>
        private static Vector3 ComputeGhostPoint(Vector3 prev, Vector3 last, Vector3 center)
        {
            float delta = Vector3.Distance(last, prev);

            Vector3 R = last - center;
            R.y = 0;

            R.Normalize();

            Vector3 T = Vector3.Cross(Vector3.up, R).normalized;

            return last + T * delta;
        }

        /// <summary>
        /// Extracts the portals (shared boundaries) between adjacent nodes in a path.
        /// </summary>
        /// <param name="path">A list of navigation nodes.</param>
        /// <returns>A list of quads (each with 4 vertices) representing the portals.</returns>
        public static List<List<Vector3>> ExtractPortals(List<OctNavGraph.GraphNode> path)
        {
            List<List<Vector3>> portals = new List<List<Vector3>>();

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

        /// <summary>
        /// Calculates the shared face (portal) between two adjacent bounding boxes if they touch.
        /// </summary>
        /// <param name="a">The first bounding box.</param>
        /// <param name="b">The second bounding box.</param>
        /// <returns>A list of 4 corner points defining the portal, or null if no valid portal exists.</returns>
        private static List<Vector3> GetTouchingPortal(Bounds a, Bounds b)
        {
            const float epsilon = 0.001f;

            Vector3 diff = b.center - a.center;
            Vector3 normal = Vector3.zero;

            // Determine the dominant axis of separation (shared face normal)
            if (Mathf.Abs(diff.x) > Mathf.Abs(diff.y) && Mathf.Abs(diff.x) > Mathf.Abs(diff.z))
            { 
                normal = Vector3.right * Mathf.Sign(diff.x); 
            }
            else if (Mathf.Abs(diff.y) > Mathf.Abs(diff.z))
            { 
                normal = Vector3.up * Mathf.Sign(diff.y);
            }
            else
            { 
                normal = Vector3.forward * Mathf.Sign(diff.z); 
            }

            // Check if they are actually touching (not overlapping)
            float distance = Vector3.Dot(diff, normal);
            float expectedDistance = Mathf.Abs(Vector3.Dot(a.extents + b.extents, normal));

            if (Mathf.Abs(distance - expectedDistance) > epsilon)
            {
                return null; // Not touching, just overlapping or separated}
            }

            // Get the touching plane position
            float planePos;
            if (normal.x != 0)
            {
                planePos = (normal.x > 0) ? (a.max.x) : (a.min.x);
            }
            else if (normal.y != 0)
            {
                planePos = (normal.y > 0) ? (a.max.y) : (a.min.y); 
            }
            else 
            { 
                planePos = (normal.z > 0) ? (a.max.z) : (a.min.z);
            }

            // Get overlap in the other 2 axes
            Vector3 aMin = a.min, aMax = a.max;
            Vector3 bMin = b.min, bMax = b.max;

            List<Vector3> quad = new List<Vector3>();

            if (normal.x != 0) // X-aligned portal
            {
                float yMin = Mathf.Max(aMin.y, bMin.y);
                float yMax = Mathf.Min(aMax.y, bMax.y);
                float zMin = Mathf.Max(aMin.z, bMin.z);
                float zMax = Mathf.Min(aMax.z, bMax.z);

                if (yMax - yMin < epsilon || zMax - zMin < epsilon) return null;
                // No meaningful overlap

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

                if (xMax - xMin < epsilon || zMax - zMin < epsilon) return null;

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

                if (xMax - xMin < epsilon || yMax - yMin < epsilon) return null;

                quad.Add(new Vector3(xMin, yMin, planePos));
                quad.Add(new Vector3(xMax, yMin, planePos));
                quad.Add(new Vector3(xMax, yMax, planePos));
                quad.Add(new Vector3(xMin, yMax, planePos));
            }

            return quad;
        }

    }
}