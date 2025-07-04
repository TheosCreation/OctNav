using UnityEditor;
using UnityEngine;

namespace OctNav
{
    [CustomEditor(typeof(OctVolume))]
    public class OctVolumeEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();

            OctVolume boundRef = (OctVolume)target;

            EditorGUILayout.Space();
            if(GUILayout.Button("Build octrees"))
            {
                boundRef.Build();
            }        
            EditorGUILayout.Space();
            if(GUILayout.Button("Reset"))
            {
                boundRef.ResetOctree();
            }
            EditorGUILayout.Space();
       
        }
        public static void BuildAllVolumes()
        {
            OctVolume[] volumes = Resources.FindObjectsOfTypeAll<OctVolume>();
            foreach (OctVolume vol in volumes)
            {
                vol.Build();
            }
        }
        public static void ResetAllVolumes()
        {
            OctVolume[] volumes = Resources.FindObjectsOfTypeAll<OctVolume>();
            foreach (OctVolume vol in volumes)
            {
                vol.ResetOctree();
            }
        }
    }
}