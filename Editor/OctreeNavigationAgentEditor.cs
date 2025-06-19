using UnityEditor;
using UnityEngine;
namespace OctNav
{
    [CustomEditor(typeof(OctreeNavigationAgent))]
    [CanEditMultipleObjects]
    public class OctreeNavigationAgentEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();
            OctreeNavigationAgent agentRef = (OctreeNavigationAgent)target;

            EditorGUILayout.Space();
            if (GUILayout.Button("Build Path"))
            {
                if(agentRef.target == null)
                {
                    Debug.LogWarning("no target set for pathfinding");
                    return;
                }
                if(OctNavigation.GetGraph(agentRef.walking) == null)
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
