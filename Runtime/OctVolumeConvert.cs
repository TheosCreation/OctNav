using Newtonsoft.Json;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

namespace OctNav
{
    [System.Serializable]
    public class SerializableVector3
    {
        public float x, y, z;

        public SerializableVector3() { }

        public SerializableVector3(Vector3 v)
        {
            x = v.x;
            y = v.y;
            z = v.z;
        }

        public Vector3 ToVector3() => new Vector3(x, y, z);
    }

    [System.Serializable]
    public class SerializableBounds
    {
        public SerializableVector3 center;
        public SerializableVector3 size;

        public SerializableBounds() { }

        public SerializableBounds(Bounds bounds)
        {
            center = new SerializableVector3(bounds.center);
            size = new SerializableVector3(bounds.size);
        }

        public Bounds ToBounds() => new Bounds(center.ToVector3(), size.ToVector3());
    }

    [System.Serializable]
    public class SerializableOctNode
    {
        public SerializableBounds bounds;
        public bool isLeaf;
        public bool hasCollision;
        public int depth;
        public List<SerializableOctNode> children = new List<SerializableOctNode>();
    }

    public static class OctVolumeConvert
    {
        public static SerializableOctNode SerializeNode(OctNode node)
        {
            SerializableOctNode data = new SerializableOctNode
            {
                bounds = new SerializableBounds(node.bounds),
                isLeaf = node.isLeaf,
                hasCollision = node.hasCollision,
                depth = node.depth
            };

            if (node.children != null)
            {
                foreach (var child in node.children)
                {
                    data.children.Add(SerializeNode(child));
                }
            }

            return data;
        }

        public static OctNode DeserializeNode(SerializableOctNode data, OctNode parent)
        {
            OctNode node = new OctNode(data.bounds.center.ToVector3(), data.bounds.size.ToVector3(), parent)
            {
                isLeaf = data.isLeaf,
                hasCollision = data.hasCollision,
                depth = data.depth
            };

            if (data.children.Count > 0)
            {
                node.children = new OctNode[8];
                for (int i = 0; i < data.children.Count; i++)
                {
                    node.children[i] = DeserializeNode(data.children[i], node);
                }
            }

            return node;
        }

        public static void EnsureDirectory(string path)
        {
            string dir = Path.GetDirectoryName(path);
            if (!Directory.Exists(dir))
            {
                Directory.CreateDirectory(dir);
            }
        }

        public static void SaveToFile(string path, OctNode root)
        {
            EnsureDirectory(path);
            var serialized = SerializeNode(root);
            var json = JsonConvert.SerializeObject(serialized, Formatting.Indented);
            File.WriteAllText(path, json);
            Debug.Log($"[OctVolume] Saved to {path}");
        }

        public static OctNode LoadFromFile(string path)
        {
            if (!File.Exists(path))
            {
                Debug.LogWarning($"[OctVolume] File not found: {path}");
                return null;
            }

            string json = File.ReadAllText(path);
            var data = JsonConvert.DeserializeObject<SerializableOctNode>(json);
            return DeserializeNode(data, null);
        }
    }
}