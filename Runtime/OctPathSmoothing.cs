using System.Collections.Generic;
using UnityEngine;
namespace OctNav { 
    public static class OctPathSmoothing 
    {
        static void ComputePortalCorners(OctNode a, OctNode b, out Vector3 portalLeft, out Vector3 portalRight)
        {
            Bounds ba = a.bounds;
            Bounds bb = b.bounds;

            Vector3 ca = ba.center;
            Vector3 cb = bb.center;

            Vector3 sa = ba.size;
            Vector3 sb = bb.size;

            float epsilon = 0.001f;

            if (Mathf.Abs(ca.x - cb.x) > Mathf.Max(sa.x, sb.x) - epsilon)
            {
                // Portal along X-axis
                float faceX = (ca.x + cb.x) * 0.5f;
                float minZ = Mathf.Max(ca.z - sa.z / 2, cb.z - sb.z / 2);
                float maxZ = Mathf.Min(ca.z + sa.z / 2, cb.z + sb.z / 2);

                portalLeft = new Vector3(faceX, 0, minZ);
                portalRight = new Vector3(faceX, 0, maxZ);
                return;
            }

            if (Mathf.Abs(ca.z - cb.z) > Mathf.Max(sa.z, sb.z) - epsilon)
            {
                // Portal along Z-axis
                float faceZ = (ca.z + cb.z) * 0.5f;
                float minX = Mathf.Max(ca.x - sa.x / 2, cb.x - sb.x / 2);
                float maxX = Mathf.Min(ca.x + sa.x / 2, cb.x + sb.x / 2);

                portalLeft = new Vector3(minX, 0, faceZ);
                portalRight = new Vector3(maxX, 0, faceZ);
                return;
            }

            if (Mathf.Abs(ca.y - cb.y) > Mathf.Max(sa.y, sb.y) - epsilon)
            {
                // Portal along Y — probably vertical transition, flatten to XZ
                float minX = Mathf.Max(ca.x - sa.x / 2, cb.x - sb.x / 2);
                float maxX = Mathf.Min(ca.x + sa.x / 2, cb.x + sb.x / 2);
                float minZ = Mathf.Max(ca.z - sa.z / 2, cb.z - sb.z / 2);
                float maxZ = Mathf.Min(ca.z + sa.z / 2, cb.z + sb.z / 2);

                float midZ = (minZ + maxZ) * 0.5f;
                portalLeft = new Vector3(minX, 0, midZ);
                portalRight = new Vector3(maxX, 0, midZ);
                return;
            }

            // Default fallback: single point
            portalLeft = ba.center;
            portalRight = ba.center;
        }

        private static float TriangleArea2D(Vector3 a, Vector3 b, Vector3 c)
        {
            return (b.x - a.x) * (c.z - a.z) - (b.z - a.z) * (c.x - a.x);
        }
        public static List<Vector3> RunFunnel(
            Vector3 startPoint,
            Vector3[] portalLeft,
            Vector3[] portalRight,
            Vector3 endPoint,
            float agentRadius)
        {
            List<Vector3> corners = new List<Vector3>();
            int leftLength = portalLeft.Length;
            if (leftLength == 0)
            {
                corners.Add(startPoint);
                corners.Add(endPoint);
                return corners;
            }

            // move in by agent radius
            for (int i = 0; i < leftLength; i++)
            {
                Vector3 L = portalLeft[i];
                Vector3 R = portalRight[i];

                Vector3 edgeDir = R - L;
                edgeDir.y = 0f;
                edgeDir.Normalize();

                Vector3 normalXZ = new Vector3(-edgeDir.z, 0f, edgeDir.x);

                portalLeft[i] = new Vector3(L.x, 0f, L.z) + normalXZ * agentRadius;
                portalRight[i] = new Vector3(R.x, 0f, R.z) - normalXZ * agentRadius;
            
            }

            Vector3 apex = startPoint;
            Vector3 left = startPoint;
            Vector3 right = startPoint;
            int apexIndex = -1;
            int leftIndex = 0;
            int rightIndex = 0;

            corners.Add(startPoint);

            for (int i = 0; i < leftLength; i++)
            {
                Vector3 nextLeft = portalLeft[i];
                Vector3 nextRight = portalRight[i];

                // try tighten right side 
                if (TriangleArea2D(apex, right, nextRight) < 0f)
                {
                    if (Equals(apex, right) || TriangleArea2D(apex, left, nextRight) > 0f)
                    {
                        // tighten
                        right = nextRight;
                        rightIndex = i;
                    }
                    else
                    {
                        // collapse right side
                        corners.Add(left);
                        apex = left;
                        apexIndex = leftIndex;
                        leftIndex = apexIndex;
                        rightIndex = apexIndex;
                        left = apex;
                        right = apex;
                        i = apexIndex + 1;
                        continue;
                    }
                }

                // try tighten left side
                if (TriangleArea2D(apex, left, nextLeft) > 0f)
                {
                    if (Equals(apex, left) || TriangleArea2D(apex, right, nextLeft) < 0f)
                    {
                        // tighten
                        left = nextLeft;
                        leftIndex = i;
                    }
                    else
                    {
                        // collapse left side
                        corners.Add(right);
                        apex = right;
                        apexIndex = rightIndex;
                        leftIndex = apexIndex;
                        rightIndex = apexIndex;
                        left = apex;
                        right = apex;
                        i = apexIndex + 1;
                        continue;
                    }
                }
                // otherwise no collapses
            }

            corners.Add(endPoint);
            return corners;
        }



        public static AgentPath BuildSmoothedPath(
            List<Node> nodePath,
            Vector3 startPos,
            Vector3 goalPos,
            float agentRadius,
            int maxSmoothingSteps)
        {
            AgentPath agentPath = new AgentPath();
            if (nodePath == null || nodePath.Count == 0)
            {
                return agentPath;
            }

            // if goal in same node go straight towards it
            if (nodePath.Count == 1)
            {
                agentPath.Add(startPos);
                agentPath.Add(goalPos);
                return agentPath;
            }

            int nodeCount = nodePath.Count;
            Vector3[] portalLeft = new Vector3[nodeCount - 1];
            Vector3[] portalRight = new Vector3[nodeCount - 1];

            //build portals between leaf pairs
            for (int i = 0; i < nodeCount - 1; i++)
            {
                OctNode a = nodePath[i].octreeNode;
                OctNode b = nodePath[i + 1].octreeNode;
                ComputePortalCorners(a, b, out Vector3 left, out Vector3 right);
                portalLeft[i] = new Vector3(left.x, 0f, left.z);
                portalRight[i] = new Vector3(right.x, 0f, right.z);
                Debug.DrawLine(left, right, Color.cyan,120);
            }
            //get 2d corners
            Vector3 startXZ = new Vector3(startPos.x, 0f, startPos.z);
            Vector3 goalXZ = new Vector3(goalPos.x, 0f, goalPos.z);
            List<Vector3> cornersXZ = RunFunnel(startXZ, portalLeft, portalRight, goalXZ, agentRadius);


            // sample ys to bring back to 3D
            List<Vector3> liftedPath = new List<Vector3>();
            foreach (Vector3 point in cornersXZ)
            {
                Vector3 lifted = point;
                foreach (var node in nodePath)
                {
                    Bounds b = node.octreeNode.bounds;
                    if (b.Contains(new Vector3(point.x, b.center.y, point.z)))
                    {
                        lifted.y = b.center.y; // or use b.min.y for foot placement
                        break;
                    }
                }
                liftedPath.Add(lifted);
            }

            if (liftedPath.Count < 3)
            {
                foreach (Vector3 w in liftedPath)
                {
                    agentPath.Add(w);
                }
                Debug.Log(agentPath.length);
                return agentPath;
            }
            // build final list and round up using Catmul-Rom
            agentPath.Add(liftedPath[0]); // start

            // agentRadius = distance to pull back along each segment

            for (int i = 1; i < liftedPath.Count - 1; i++)
            {
                Vector3 prev = liftedPath[i - 1];
                Vector3 curr = liftedPath[i];
                Vector3 next = liftedPath[i + 1];

                // compute XZ directions (ignore Y)
                Vector3 dir1 = curr - prev;
                dir1.y = 0f;
                dir1.Normalize();

                Vector3 dir2 = next - curr;
                dir2.y = 0f;
                dir2.Normalize();

                Vector3 A = curr - dir1 * agentRadius;
                Vector3 B = curr + dir2 * agentRadius;

                agentPath.Add(A);

                for (int s = 1; s < maxSmoothingSteps; s++)
                {
                    float t = (float)s / maxSmoothingSteps;
                    Vector3 pt = CatmullRom(prev, A, B, next, t);
                    agentPath.Add(pt);
                }

                agentPath.Add(B);
            }

            // add final goal
            agentPath.Add(liftedPath[liftedPath.Count - 1]);
            Debug.Log(agentPath.length);
            return agentPath;
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
    }
}