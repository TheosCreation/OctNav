using UnityEditor;
using UnityEditor.IMGUI.Controls;
using UnityEngine;

namespace OctNav
{
    [CustomEditor(typeof(BoundsHandles))]
    public class BoundsEditor : Editor
    {
        BoxBoundsHandle boundsHandle = new BoxBoundsHandle();
        public override bool RequiresConstantRepaint()
        {
            return true;
        }
        void OnSceneGUI()
        {

            BoundsHandles boundRef = (BoundsHandles)target;
            Transform t = boundRef.transform;

            boundsHandle.center = boundRef.center;
            boundsHandle.size = boundRef.size;
            Matrix4x4 handleMatrix = Matrix4x4.TRS(t.position, t.rotation, t.lossyScale);
            using (new Handles.DrawingScope(handleMatrix))
            {
                EditorGUI.BeginChangeCheck();
                boundsHandle.DrawHandle();

                if (EditorGUI.EndChangeCheck())
                {
                    Undo.RecordObject(boundRef, "Edit Bounds");

                    boundRef.center = boundsHandle.center;
                    boundRef.size = boundsHandle.size;

                    boundRef.OnBoundsChanged?.Invoke(boundRef.GetBounds());
                    EditorUtility.SetDirty(boundRef);
                    SceneView.RepaintAll();
                }
            }
        }
    }


}