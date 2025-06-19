using Newtonsoft.Json;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
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

    [Serializable]
    public class SerializableOctVolumeData
    {
        public SerializableOctNode root;
        public List<int> allNodeIds;
        public List<int> emptyLeafIds;
        public List<int> hitNodesIds;
        public Dictionary<int, int[]> faceLinkIds;
        public List<SerializableEdge> graphEdges;
        public List<SerializableEdge> groundGraphEdges;

    }

    [Serializable]
    public class SerializableEdge
    {
        public int a, b;
        public SerializableEdge(int a, int b) { this.a = a; this.b = b; }
    }

    [System.Serializable]
    public class SerializableOctNode
    {
        public int id;
        public SerializableBounds bounds;
        public bool isLeaf;
        public bool hasCollision;
        public bool isOutside;
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
                isOutside = node.isOutside,
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
                isOutside = data.isOutside,
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
                //Debug.LogWarning($"[OctVolume] File not found: {path}");
                return null;
            }

            string json = File.ReadAllText(path);
            var data = JsonConvert.DeserializeObject<SerializableOctNode>(json);
            return DeserializeNode(data, null);
        }


        public static void SaveFullVolume(string path, OctVolume vol)
        {
            SerializableOctNode rootData = SerializeNode(vol.root);

            List<OctNode> allNodes = vol.allNodes;
            List<OctNode> emptyLeaves = vol.emptyLeaves;
            List<OctNode> hitNodes = vol.hitNodes;
            Dictionary<int,OctNode> lookup = allNodes.ToDictionary(n => n.id, n => n);

            Dictionary<int, int[]> faceLinks = new Dictionary<int, int[]>();
            foreach (OctNode node in allNodes)
            {
                int[] fl = new int[6];
                for (int i = 0; i < 6; i++)
                {
                    fl[i] = node.faceLinks[i]?.id ?? -1;
                }
                faceLinks[node.id] = fl;
            }

            List<SerializableEdge> graphEdges = OctNavigation.graph.edges
                .Select(e => new SerializableEdge(e.a.octreeNode.id, e.b.octreeNode.id))
                .ToList();

            List<SerializableEdge> groundEdges = OctNavigation.groundGraph.edges
                .Select(e => new SerializableEdge(e.a.octreeNode.id, e.b.octreeNode.id))
                .ToList();

            SerializableOctVolumeData container = new SerializableOctVolumeData
            {
                root = rootData,
                allNodeIds = allNodes.Select(n => n.id).ToList(),
                emptyLeafIds = emptyLeaves.Select(n => n.id).ToList(),
                hitNodesIds = hitNodes.Select(n => n.id).ToList(),
                faceLinkIds = faceLinks,
                graphEdges = graphEdges,
                groundGraphEdges = groundEdges
            };

            string json = JsonConvert.SerializeObject(container, Formatting.Indented);
            File.WriteAllText(path, json);

        }
        public static bool LoadFullVolume(string path, OctVolume vol)
        {
            if (!File.Exists(path)) return false;
            SerializableOctVolumeData container = JsonConvert.DeserializeObject<SerializableOctVolumeData>(File.ReadAllText(path));

            Dictionary<int, OctNode> idMap = new Dictionary<int, OctNode>();
            OctNode Recurse(SerializableOctNode data, OctNode parent)
            {
                OctNode node = new OctNode(data.bounds.center.ToVector3(), data.bounds.size.ToVector3(), parent)
                {
                    isLeaf = data.isLeaf,
                    hasCollision = data.hasCollision,
                    isOutside = data.isOutside,
                    depth = data.depth,
                };
                typeof(OctNode).GetField("id", BindingFlags.Instance | BindingFlags.NonPublic)
                                .SetValue(node, data.id);
                idMap[node.id] = node;
                foreach (SerializableOctNode childData in data.children)
                {
                    node.children = node.children ?? new OctNode[8];
                    OctNode child = Recurse(childData, node);
                    node.children[child.id] = child;
                }
                return node;
            }
            vol.root = Recurse(container.root, null);

            foreach (var kv in container.faceLinkIds)
            {
                OctNode node = idMap[kv.Key];
                for (int i = 0; i < 6; i++)
                {
                    int nid = kv.Value[i];
                    node.faceLinks[i] = nid >= 0 ? idMap[nid] : null;
                }
            }

            vol.allNodes = container.allNodeIds.Select(id => idMap[id]).ToList();
            vol.emptyLeaves = container.emptyLeafIds.Select(id => idMap[id]).ToList();
            vol.hitNodes = container.hitNodesIds.Select(id => idMap[id]).ToList();


            vol.BuildGraph();
            vol.BuildWalkableGraph();

            return true;
        }

    }
}