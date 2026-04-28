using UnityEditor;
using UnityEngine;
namespace OctNav
{
    [ExecuteAlways]
    public class BoundsHandles : MonoBehaviour
    {
        public Vector3 center = Vector3.zero;
        public Vector3 size = Vector3.one * 10f;
        // basically just a c++ function pointer
        public delegate void BoundsChanged(Bounds newBounds);
        public BoundsChanged OnBoundsChanged;
        public Bounds GetBounds() => new Bounds(center, size);

        public void SetBounds(Vector3 newCenter, Vector3 newSize)
        {
            center = newCenter;
            size = newSize;
        }

        private void OnValidate()
        {
            if (OnBoundsChanged != null)
            {
                OnBoundsChanged.Invoke(GetBounds());



            }
#if UNITY_EDITOR
        EditorUtility.SetDirty(this);
        SceneView.RepaintAll();
#endif
        }

    }

}