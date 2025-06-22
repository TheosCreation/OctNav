using System.Collections.Generic;
using System.IO;
using System.Linq;
using System;
using UnityEngine;
using Unity.Plastic.Newtonsoft.Json;

namespace OctNav
{
    /// <summary>
    /// Serializable wrapper for UnityEngine.Vector3 to support JSON serialization.
    /// </summary>
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

    /// <summary>
    /// Serializable wrapper for UnityEngine.Bounds using SerializableVector3.
    /// </summary>
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

    /// <summary>
    /// Represents a full serialized OctVolume including root node and key node lists.
    /// </summary>
    public class SerializableOctVolumeData
    {
        public SerializableOctNode root;
        public List<int> allNodeIds;
        public List<int> emptyLeafIds;
        public List<int> hitNodesIds;
    }

    /// <summary>
    /// Represents a serialized edge connection between two node IDs.
    /// </summary>
    [Serializable]
    public class SerializableEdge
    {
        public int a, b;
        public SerializableEdge(int a, int b) { this.a = a; this.b = b; }
    }

    /// <summary>
    /// Serializable representation of an OctNode, including bounds and face connections.
    /// </summary>
    [System.Serializable]
    public class SerializableOctNode
    {
        public int id;
        public SerializableBounds bounds;
        public bool isLeaf;
        public bool hasCollision;
        public bool isOutside;
        public int depth;
        public List<int> faceLinkIds = new List<int>();
        public List<SerializableOctNode> children = new List<SerializableOctNode>();
    }

    /// <summary>
    /// Handles serialization and deserialization of OctNode trees and OctVolume state to JSON files.
    /// </summary>
    public static class OctVolumeConvert
    {
        static Dictionary<int, OctNode> idMap = new Dictionary<int, OctNode>();

        /// <summary>
        /// Recursively serializes an OctNode and its children into a SerializableOctNode.
        /// </summary>
        public static SerializableOctNode SerializeNode(OctNode node)
        {
            SerializableOctNode data = new SerializableOctNode
            {
                id = node.id,
                bounds = new SerializableBounds(node.bounds),
                isLeaf = node.isLeaf,
                hasCollision = node.hasCollision,
                isOutside = node.isOutside,
                depth = node.depth,
                faceLinkIds = node.faceLinks.Select(n => n != null ? n.id : -1).ToList()
            };

            if (node.children != null)
            {
                foreach (OctNode child in node.children)
                {
                    data.children.Add(SerializeNode(child));
                }
            }

            return data;
        }

        /// <summary>
        /// Recursively deserializes a SerializableOctNode into an OctNode, rebuilding hierarchy.
        /// </summary>
        public static OctNode DeserializeNode(SerializableOctNode data, OctNode parent)
        {
            OctNode node = new OctNode(data.id, data.bounds.center.ToVector3(), data.bounds.size.ToVector3(), parent)
            {
                isLeaf = data.isLeaf,
                hasCollision = data.hasCollision,
                isOutside = data.isOutside,
                depth = data.depth
            };

            idMap.Add(data.id, node);

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

        /// <summary>
        /// Ensures the target directory exists before saving.
        /// </summary>
        public static void EnsureDirectory(string path)
        {
            string dir = Path.GetDirectoryName(path);
            if (!Directory.Exists(dir))
            {
                Directory.CreateDirectory(dir);
            }
        }

        /// <summary>
        /// Saves a single OctNode hierarchy to a JSON file.
        /// </summary>
        public static void SaveToFile(string path, OctNode root)
        {
            EnsureDirectory(path);
            SerializableOctNode serialized = SerializeNode(root);
            string json = JsonConvert.SerializeObject(serialized, Formatting.Indented);
            File.WriteAllText(path, json);
            Debug.Log($"[OctVolume] Saved to {path}");
        }

        /// <summary>
        /// Loads an OctNode hierarchy from a JSON file.
        /// </summary>
        public static OctNode LoadFromFile(string path)
        {
            if (!File.Exists(path)) return null;

            string json = File.ReadAllText(path);
            SerializableOctNode data = JsonConvert.DeserializeObject<SerializableOctNode>(json);
            return DeserializeNode(data, null);
        }

        /// <summary>
        /// Saves a full OctVolume including node metadata and hierarchy.
        /// </summary>
        public static void SaveFullVolume(string path, OctVolume vol)
        {
            SerializableOctNode serializedRoot = SerializeNode(vol.root);

            SerializableOctVolumeData container = new SerializableOctVolumeData
            {
                root = serializedRoot,
                allNodeIds = vol.allNodes.Select(n => n.id).ToList(),
                emptyLeafIds = vol.emptyLeaves.Select(n => n.id).ToList(),
                hitNodesIds = vol.hitNodes.Select(n => n.id).ToList()
            };

            System.Threading.Tasks.Task.Run(() =>
            {
                try
                {
                    string json = JsonConvert.SerializeObject(container, Formatting.Indented);

                    string dir = Path.GetDirectoryName(path);
                    if (!Directory.Exists(dir)) Directory.CreateDirectory(dir);

                    File.WriteAllText(path, json);
                    //Debug.Log($"[OctVolume] saved to {path}");
                }
                catch (System.Exception ex)
                {
                    Debug.LogError($"[OctVolume] Async save failed: {ex.Message}");
                }
            });
        }


        /// <summary>
        /// Loads a full OctVolume including node metadata, connectivity, and root structure.
        /// </summary>
        public static bool LoadFullVolume(string path, OctVolume vol)
        {
            if (!File.Exists(path)) return false;

            idMap.Clear();

            SerializableOctVolumeData container = JsonConvert.DeserializeObject<SerializableOctVolumeData>(File.ReadAllText(path));
            vol.root = DeserializeNode(container.root, null);

            AssignFaceLinks(container.root);

            vol.allNodes = container.allNodeIds.Where(idMap.ContainsKey).Select(id => idMap[id]).ToList();
            vol.emptyLeaves = container.emptyLeafIds.Where(idMap.ContainsKey).Select(id => idMap[id]).ToList();
            vol.hitNodes = container.hitNodesIds.Where(idMap.ContainsKey).Select(id => idMap[id]).ToList();

            return true;
        }

        /// <summary>
        /// Recursively assigns face neighbor links based on stored node IDs.
        /// </summary>
        static void AssignFaceLinks(SerializableOctNode sNode)
        {
            if (idMap.TryGetValue(sNode.id, out OctNode realNode) && sNode.faceLinkIds != null)
            {
                for (int i = 0; i < sNode.faceLinkIds.Count && i < 6; i++)
                {
                    int linkId = sNode.faceLinkIds[i];
                    realNode.faceLinks[i] = linkId >= 0 && idMap.TryGetValue(linkId, out OctNode linkedNode) ? linkedNode : null;
                }
            }

            foreach (SerializableOctNode child in sNode.children)
            {
                AssignFaceLinks(child);
            }
        }
    }
}