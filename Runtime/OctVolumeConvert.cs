using System.Collections.Generic;
using System.IO;
using System.Linq;
using System;
using System.Text;
using UnityEngine;

namespace OctNav
{
    /// <summary>
    /// Handles binary serialization and deserialization of OctNode trees and OctVolume state.
    /// Format is versioned and little-endian.
    /// </summary>
    public static class OctVolumeConvert
    {
        // File tags
        const int MAGIC_NODE = 0x4F435456; // 'OCTV'
        const int MAGIC_VOL = 0x4F435646; // 'OCVF'
        const int VERSION = 1;

        static Dictionary<int, OctNode> idMap = new Dictionary<int, OctNode>(1024);
        static Dictionary<int, int[]> pendingFaceLinks = new Dictionary<int, int[]>(1024);

        // -------- Core binary helpers --------

        static void WriteVec3(BinaryWriter bw, Vector3 v)
        {
            bw.Write(v.x); bw.Write(v.y); bw.Write(v.z);
        }

        static Vector3 ReadVec3(BinaryReader br)
        {
            float x = br.ReadSingle();
            float y = br.ReadSingle();
            float z = br.ReadSingle();
            return new Vector3(x, y, z);
        }

        static void WriteBounds(BinaryWriter bw, Bounds b)
        {
            WriteVec3(bw, b.center);
            WriteVec3(bw, b.size);
        }

        static Bounds ReadBounds(BinaryReader br)
        {
            var c = ReadVec3(br);
            var s = ReadVec3(br);
            return new Bounds(c, s);
        }

        // -------- Node I/O --------

        static void WriteNode(BinaryWriter bw, OctNode node)
        {
            // Header for this node
            bw.Write(node.id);
            WriteBounds(bw, node.bounds);
            bw.Write(node.isLeaf);
            bw.Write(node.hasCollision);
            bw.Write(node.isOutside);
            bw.Write(node.depth);

            // Face links as IDs (6 ints)
            for (int i = 0; i < 6; i++)
            {
                int id = (node.faceLinks != null && i < node.faceLinks.Length && node.faceLinks[i] != null)
                    ? node.faceLinks[i].id
                    : -1;
                bw.Write(id);
            }

            // Children presence mask (8 bools), then recursively write present children in index order
            bool hasChildren = node.children != null && node.children.Length == 8;
            for (int i = 0; i < 8; i++)
            {
                bw.Write(hasChildren && node.children[i] != null);
            }
            if (hasChildren)
            {
                for (int i = 0; i < 8; i++)
                {
                    if (node.children[i] != null)
                        WriteNode(bw, node.children[i]);
                }
            }
        }

        static OctNode ReadNode(BinaryReader br, OctNode parent)
        {
            int id = br.ReadInt32();
            Bounds bounds = ReadBounds(br);

            // Construct node
            var node = new OctNode(id, bounds.center, bounds.size, parent);

            node.isLeaf = br.ReadBoolean();
            node.hasCollision = br.ReadBoolean();
            node.isOutside = br.ReadBoolean();
            node.depth = br.ReadInt32();

            // Prepare face links
            if (node.faceLinks == null || node.faceLinks.Length != 6)
                node.faceLinks = new OctNode[6];

            // Capture face link IDs for later resolution
            int[] six = new int[6];
            for (int i = 0; i < 6; i++) six[i] = br.ReadInt32();
            pendingFaceLinks[id] = six;

            // Read children presence mask
            bool[] present = new bool[8];
            for (int i = 0; i < 8; i++) present[i] = br.ReadBoolean();

            // Recurse
            node.children = new OctNode[8];
            for (int i = 0; i < 8; i++)
            {
                if (present[i])
                    node.children[i] = ReadNode(br, node);
                else
                    node.children[i] = null;
            }

            // Map ID
            idMap[id] = node;
            return node;
        }

        static void ResolveAllFaceLinks()
        {
            if (pendingFaceLinks == null || pendingFaceLinks.Count == 0) return;

            foreach (var kv in pendingFaceLinks)
            {
                int nodeId = kv.Key;
                int[] links = kv.Value;

                if (!idMap.TryGetValue(nodeId, out var node)) continue;

                if (node.faceLinks == null || node.faceLinks.Length != 6)
                    node.faceLinks = new OctNode[6];

                for (int i = 0; i < 6; i++)
                {
                    int linkId = links[i];
                    node.faceLinks[i] = (linkId >= 0 && idMap.TryGetValue(linkId, out var neigh)) ? neigh : null;
                }
            }

            pendingFaceLinks.Clear();
        }

        // -------- Filesystem helpers --------

        public static void EnsureDirectory(string path)
        {
            string dir = Path.GetDirectoryName(path);
            if (!string.IsNullOrEmpty(dir) && !Directory.Exists(dir))
                Directory.CreateDirectory(dir);
        }

        // -------- Public API: node-only files --------

        public static void SaveToFile(string path, OctNode root)
        {
            EnsureDirectory(path);
            using (var fs = new FileStream(path, FileMode.Create, FileAccess.Write, FileShare.None, 1 << 20))
            using (var bw = new BinaryWriter(fs, Encoding.UTF8, false))
            {
                bw.Write(MAGIC_NODE);
                bw.Write(VERSION);
                WriteNode(bw, root);
            }
            Debug.Log("[OctVolume] Saved binary node to " + path);
        }

        public static OctNode LoadFromFile(string path)
        {
            if (!File.Exists(path)) return null;

            idMap.Clear();
            pendingFaceLinks.Clear();

            using (var fs = new FileStream(path, FileMode.Open, FileAccess.Read, FileShare.Read, 1 << 20, FileOptions.SequentialScan))
            using (var br = new BinaryReader(fs, Encoding.UTF8, false))
            {
                int magic = br.ReadInt32();
                int version = br.ReadInt32();
                if (magic != MAGIC_NODE) throw new InvalidDataException("Bad magic for node file.");
                if (version != VERSION) throw new NotSupportedException("Unsupported version.");

                var root = ReadNode(br, null);
                ResolveAllFaceLinks();
                return root;
            }
        }

        // -------- Public API: full volume files --------

        public static void SaveFullVolume(string path, OctVolume vol)
        {
            EnsureDirectory(path);

            System.Threading.Tasks.Task.Run(() =>
            {
                try
                {
                    using (var fs = new FileStream(path, FileMode.Create, FileAccess.Write, FileShare.None, 1 << 20))
                    using (var bw = new BinaryWriter(fs, Encoding.UTF8, false))
                    {
                        bw.Write(MAGIC_VOL);
                        bw.Write(VERSION);

                        WriteBounds(bw, vol.bounds);
                        
                        // Root tree
                        WriteNode(bw, vol.root);

                        // allNodeIds
                        int allCount = vol.allNodes != null ? vol.allNodes.Count : 0;
                        bw.Write(allCount);
                        if (allCount > 0)
                        {
                            for (int i = 0; i < allCount; i++) bw.Write(vol.allNodes[i].id);
                        }

                        // emptyLeafIds
                        int emptyCount = vol.emptyLeaves != null ? vol.emptyLeaves.Count : 0;
                        bw.Write(emptyCount);
                        if (emptyCount > 0)
                        {
                            for (int i = 0; i < emptyCount; i++) bw.Write(vol.emptyLeaves[i].id);
                        }

                        // hitNodesIds
                        int hitCount = vol.hitNodes != null ? vol.hitNodes.Count : 0;
                        bw.Write(hitCount);
                        if (hitCount > 0)
                        {
                            for (int i = 0; i < hitCount; i++) bw.Write(vol.hitNodes[i].id);
                        }
                    }
                }
                catch (Exception ex)
                {
                    Debug.LogError("[OctVolume] Async binary save failed: " + ex.Message);
                }
            });
        }

        public static bool LoadFullVolume(string path, OctVolume vol)
        {
            if (vol == null)
            {
                Debug.LogError("[OctVolume] Load failed: OctVolume is null");
                return false;
            }
        
            if (string.IsNullOrEmpty(path))
            {
                Debug.LogError("[OctVolume] Load failed: path is null or empty");
                return false;
            }
        
            if (!File.Exists(path))
            {
                Debug.LogWarning("[OctVolume] Volume file does not exist: " + path);
                return false;
            }
        
            try
            {
                idMap.Clear();
                pendingFaceLinks.Clear();
        
                using (FileStream fileStream =
                       new FileStream(
                           path,
                           FileMode.Open,
                           FileAccess.Read,
                           FileShare.Read,
                           1 << 20,
                           FileOptions.SequentialScan))
                using (BinaryReader binaryReader =
                       new BinaryReader(fileStream, Encoding.UTF8, false))
                {
                    int magic = binaryReader.ReadInt32();
                    int version = binaryReader.ReadInt32();
        
                    if (magic != MAGIC_VOL)
                    {
                        Debug.LogError("[OctVolume] Invalid magic header in volume file: " + path);
                        return false;
                    }
        
                    if (version != VERSION)
                    {
                        Debug.LogError(
                            "[OctVolume] Unsupported volume version. Expected " +
                            VERSION + " but got " + version
                        );
                        return false;
                    }
        
                    // Bounds
                    Bounds bounds = ReadBounds(binaryReader);
                    vol.bounds = bounds;
                    vol.boundHandles.SetBounds(bounds.center, bounds.size);
        
                    // Root
                    vol.root = ReadNode(binaryReader, null);
                    ResolveAllFaceLinks();
        
                    // allNodes
                    int allCount = Mathf.Max(0, binaryReader.ReadInt32());
                    if (vol.allNodes == null)
                    {
                        vol.allNodes = new List<OctNode>(allCount);
                    }
                    else
                    {
                        vol.allNodes.Clear();
                        if (vol.allNodes.Capacity < allCount)
                        {
                            vol.allNodes.Capacity = allCount;
                        }
                    }
        
                    for (int i = 0; i < allCount; i++)
                    {
                        int id = binaryReader.ReadInt32();
                        if (idMap.TryGetValue(id, out OctNode node))
                        {
                            vol.allNodes.Add(node);
                        }
                    }
        
                    // emptyLeaves
                    int emptyCount = Mathf.Max(0, binaryReader.ReadInt32());
                    if (vol.emptyLeaves == null)
                    {
                        vol.emptyLeaves = new List<OctNode>(emptyCount);
                    }
                    else
                    {
                        vol.emptyLeaves.Clear();
                        if (vol.emptyLeaves.Capacity < emptyCount)
                        {
                            vol.emptyLeaves.Capacity = emptyCount;
                        }
                    }
        
                    for (int i = 0; i < emptyCount; i++)
                    {
                        int id = binaryReader.ReadInt32();
                        if (idMap.TryGetValue(id, out OctNode node))
                        {
                            vol.emptyLeaves.Add(node);
                        }
                    }
        
                    // hitNodes
                    int hitCount = Mathf.Max(0, binaryReader.ReadInt32());
                    if (vol.hitNodes == null)
                    {
                        vol.hitNodes = new List<OctNode>(hitCount);
                    }
                    else
                    {
                        vol.hitNodes.Clear();
                        if (vol.hitNodes.Capacity < hitCount)
                        {
                            vol.hitNodes.Capacity = hitCount;
                        }
                    }
        
                    for (int i = 0; i < hitCount; i++)
                    {
                        int id = binaryReader.ReadInt32();
                        if (idMap.TryGetValue(id, out OctNode node))
                        {
                            vol.hitNodes.Add(node);
                        }
                    }
                }
        
                Debug.Log("[OctVolume] Loaded volume successfully: " + path);
                return true;
            }
            catch (Exception ex)
            {
                Debug.LogError(
                    "[OctVolume] Failed to load volume file: " + path +
                    "\n" + ex
                );
                return false;
            }
        }
    }
}
