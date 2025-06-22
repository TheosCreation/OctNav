using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine.SceneManagement;

namespace OctNav
{

    [InitializeOnLoad]
    public static class OctNavigationAutoLoader
    {
        static OctNavigationAutoLoader()
        {
            // Called when Unity Editor loads
            EditorApplication.update += OnEditorStartup;
            // Called when a new scene is loaded
            EditorSceneManager.sceneOpened += OnSceneOpened;
        }

        private static void OnEditorStartup()
        {
            EditorApplication.update -= OnEditorStartup;
            OctManager.TryReloadOctreeVolumes();
        }

        private static void OnSceneOpened(Scene scene, OpenSceneMode mode)
        {
            OctManager.TryReloadOctreeVolumes();
        }
    }


}