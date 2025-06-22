using UnityEditor;
using UnityEngine;
namespace OctNav
{
    [CustomEditor(typeof(OctAgent))]
    [CanEditMultipleObjects]
    public class OctNavigationAgentEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();
            OctAgent agentRef = (OctAgent)target;

            EditorGUILayout.Space();
            if (GUILayout.Button("Build Path"))
            {
                if(agentRef.target == null)
                {
                    Debug.LogWarning("no target set for pathfinding");
                    return;
                }
                if(OctManager.GetGraph(agentRef.walking) == null)
                {
                    Debug.LogWarning("no octree graph calculated for pathfinding");
                    return;
                }

                agentRef.SetTarget();
            }
            EditorGUILayout.Space();
            if (GUILayout.Button("Reset Path"))
            {
                agentRef.ResetPath();
            }
        }
    }
}
