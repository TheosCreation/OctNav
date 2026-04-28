using System.Diagnostics;
using UnityEditor;
using UnityEngine;
using NaughtyAttributes.Editor;
using UnityEngine.Profiling;
using Debug = UnityEngine.Debug;

namespace OctNav
{
    [CustomEditor(typeof(OctAgent))]
    [CanEditMultipleObjects]
    public class OctAgentEditor : NaughtyInspector
    {
        public override void OnInspectorGUI()
        {
            base.OnInspectorGUI();
            OctAgent agentRef = (OctAgent)target;

            EditorGUILayout.Space();
            if (GUILayout.Button("Build Path"))
            {   
                if(agentRef.target == null)
                {
                    Debug.LogWarning("no target set for pathfinding");
                    return;
                }
                if (agentRef.walking)
                {
                    if (!agentRef.InitNavMeshAgent())
                    {
                        return;
                    }
                }
                if (OctManager.GetGraph(agentRef.walking) == null)
                {
                    Debug.LogWarning("no octree graph calculated for pathfinding");
                    return;
                }
                Debug.Log("setting path to target: " + agentRef.target.name); 
                agentRef.SetTarget();
                agentRef.BeginMovement();
                agentRef.GetNewPath();
               
            }
            EditorGUILayout.Space();
            if (GUILayout.Button("Reset Path"))
            {
                agentRef.ClearPath();
            }
        }
    }
}
