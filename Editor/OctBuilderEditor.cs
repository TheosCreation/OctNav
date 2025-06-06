using UnityEditor;
using UnityEngine;

namespace OctNav
{
    [CustomEditor(typeof(OctBuilder))]
    public class OctBuilderEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();

            OctBuilder builder = (OctBuilder)target;

            EditorGUILayout.Space();
            if(GUILayout.Button("Build"))
            {
                builder.Build();
            }
        }
    }
}