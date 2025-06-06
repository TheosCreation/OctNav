using UnityEngine;

namespace OctNav
{
    public static class OctNavigation
    {
        public static Graph graph = new Graph();
        private static Graph tempGraph = new Graph();

        // functions for user to build small sections of navigational space
        public static void BuildNavigation(Vector3 position, Vector3 extent)
        {
            Bounds buildBounds = new Bounds(position, extent);
            BuildNavigation(buildBounds);
        }
        public static void BuildNavigation(Bounds bounds)
        {
            OctVolume[] octVolumes = Object.FindObjectsByType<OctVolume>(FindObjectsSortMode.None);
            foreach (OctVolume volume in octVolumes)
            {
                if (volume != null)
                {
                    if (bounds.Intersects(volume.bounds))
                    {
                        volume.BuildSection(bounds);
                    }
                }
            }
        }

#if UNITY_EDITOR
        static OctNavigation()
        {
            UnityEditor.EditorApplication.playModeStateChanged += OnPlayModeChanged;
        }

        private static void OnPlayModeChanged(UnityEditor.PlayModeStateChange state)
        {
            if (state == UnityEditor.PlayModeStateChange.EnteredEditMode)
            {
                TryReloadOctreeVolumes();
            }
        }

        private static void TryReloadOctreeVolumes()
        {
            OctVolume[] octVolumes = Object.FindObjectsByType<OctVolume>(FindObjectsSortMode.None);
            foreach (OctVolume volume in octVolumes)
            {
                if (volume != null)
                {
                    Debug.Log($"[OctNavigation] Reloading Octree for: {volume.name}");
                    if (!volume.LoadOctree())
                    {
                        Debug.LogError("Error loading octree data");
                    }
                }
            }
        }
#endif
    }
}
