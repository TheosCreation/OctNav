using UnityEngine;

namespace OctNav
{
    public static class OctNavigation
    {
        public static Graph graph = new Graph();
        public static Graph groundGraph = new Graph();

        public static Graph GetGraph(bool grounded = false)
        {
            return grounded ? groundGraph : graph;
        }
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

        public static void TryReloadOctreeVolumes()
        {
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
