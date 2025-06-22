using System.Collections.Generic;
using UnityEngine;

namespace OctNav
{
    /// <summary>
    /// Static utility class for managing and building A* navigation graphs using octree volumes.
    /// </summary>
    public static class OctManager
    {
        /// <summary>
        /// Main navigation graph used for general pathfinding.
        /// </summary>
        public static OctNavGraph graph = new OctNavGraph();
        
        /// <summary>
        /// Separate graph used for ground-based navigation constraints.
        /// </summary>
        public static OctNavGraph groundGraph = new OctNavGraph();

        /// <summary>
        /// Returns either the default graph or ground-constrained graph.
        /// </summary>
        /// <param name="grounded">Whether to return the ground-specific graph.</param>
        public static OctNavGraph GetGraph(bool grounded = false)
        {
            return grounded ? groundGraph : graph;
        }

        /// <summary>
        /// Builds the navigation graph using a position and extent (size).
        /// </summary>
        /// <param name="position">Center of the bounds.</param>
        /// <param name="extent">Size of the bounds.</param>
        public static void BuildNavigation(Vector3 position, Vector3 extent)
        {
            Bounds buildBounds = new Bounds(position, extent);
            BuildNavigation(buildBounds);
        }

        /// <summary>
        /// Builds navigation data by scanning all <see cref="OctVolume"/>s that intersect with the given bounds.
        /// </summary>
        /// <param name="bounds">The bounds to build navigation within.</param>
        public static void BuildNavigation(Bounds bounds)
        {
            OctVolume[] octVolumes = Object.FindObjectsByType<OctVolume>(FindObjectsSortMode.None);
            foreach (OctVolume volume in octVolumes)
            {
                if (volume != null && bounds.Intersects(volume.bounds))
                {
                    volume.BuildSection(bounds);
                }
            }
        }

        public static int GetAllLeafNodesCount()
        {
            int count = 0;
            foreach (var vol in Resources.FindObjectsOfTypeAll<OctVolume>())
            {
                vol.emptyLeaves.Clear();
                vol.GetEmptyLeaves(vol.root);
                count += vol.emptyLeaves.Count;
            }
            return count;
        }

        public static List<OctNode> GetAllLeafNodesInScene()
        {
            List<OctNode> leaves = new List<OctNode>();
            foreach (var vol in Resources.FindObjectsOfTypeAll<OctVolume>())
            {
                vol.emptyLeaves.Clear();
                vol.GetEmptyLeaves(vol.root);
                leaves.AddRange(vol.emptyLeaves);
            }
            return leaves;
        }

#if UNITY_EDITOR
        /// <summary>
        /// Static constructor subscribes to editor play mode state changes.
        /// </summary>
        static OctManager()
        {
            UnityEditor.EditorApplication.playModeStateChanged += OnPlayModeChanged;
        }

        /// <summary>
        /// Called when Unity's play mode changes; reloads octree volumes when entering edit mode.
        /// </summary>
        /// <param name="state">The new play mode state.</param>
        private static void OnPlayModeChanged(UnityEditor.PlayModeStateChange state)
        {
            if (state == UnityEditor.PlayModeStateChange.EnteredEditMode)
            {
                TryReloadOctreeVolumes();
            }
        }

        /// <summary>
        /// Reloads all <see cref="OctVolume"/> components in the scene by invoking their LoadOctree method.
        /// Useful for restoring serialized data after exiting play mode.
        /// </summary>
        public static void TryReloadOctreeVolumes()
        {
            graph = new OctNavGraph();
            groundGraph = new OctNavGraph();

            OctVolume[] octVolumes = Object.FindObjectsByType<OctVolume>(FindObjectsSortMode.None);
            foreach (OctVolume volume in octVolumes)
            {
                if (volume != null)
                {
                    volume.LoadOctree();
                }
            }
        }
#endif
    }
}