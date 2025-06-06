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
                boundRef.BuildChildren();
            }        
            EditorGUILayout.Space();
            if(GUILayout.Button("Reset"))
            {
                boundRef.Reset();
            }
            EditorGUILayout.Space();
       
        }
        public static void BuildAllVolumes()
        {
            OctVolume[] volumes = Resources.FindObjectsOfTypeAll<OctVolume>();
            foreach (OctVolume vol in volumes)
            {
                vol.BuildChildren();
            }
        }
        public static void ResetAllVolumes()
        {
            OctVolume[] volumes = Resources.FindObjectsOfTypeAll<OctVolume>();
            foreach (OctVolume vol in volumes)
            {
                vol.Reset();
            }
        }
    }
}