using UnityEngine;

namespace OctNav
{
    [RequireComponent(typeof(BoundsHandles))]
    public class OctBuilder : MonoBehaviour
    {
        [HideInInspector] public BoundsHandles boundHandles;
        [HideInInspector] public Bounds bounds;
        private void OnValidate()
        {
            if (boundHandles == null)
            {
                boundHandles = GetComponent<BoundsHandles>();
            }
            boundHandles.OnBoundsChanged -= UpdateBounds;
            boundHandles.OnBoundsChanged += UpdateBounds;
            bounds = boundHandles.GetBounds();

        }
        private void UpdateBounds(Bounds newBounds)
        {
            bounds = newBounds;
        }
        public void Build()
        {
            OctNavigation.BuildNavigation(bounds);
        }
    }
}