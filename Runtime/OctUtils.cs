using System.Security.Cryptography;
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
        public static Color Color(this OctColour colour)
        {
            return colours[(int)colour];
        }
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

    }
}