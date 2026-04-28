#if UNITY_EDITOR
using System;
using System.IO;
using UnityEditor;
using UnityEditor.Build;
using UnityEditor.Build.Reporting;
using UnityEngine;

namespace OctNav
{
    public sealed class CopySceneDataPostBuild : IPostprocessBuildWithReport
    {
        public int callbackOrder
        {
            get
            {
                return 0;
            }
        }

        public void OnPostprocessBuild(BuildReport report)
        {
            try
            {
                string source = OctVolume.ExternalSceneDataRoot;
                if (!Directory.Exists(source))
                {
                    Debug.LogWarning("[OctNav] No octree data at " + source + ". Nothing to copy.");
                    return;
                }

                string buildDir = GetBuildDirectory(report);
                string dest = Path.Combine(buildDir, OctVolume.SceneDataFolderName, OctVolume.OctreeSubfolderName);

                if (Directory.Exists(dest))
                {
                    try
                    {
                        Directory.Delete(dest, true);
                    }
                    catch (Exception ex)
                    {
                        Debug.LogError("[OctNav] Failed to clear destination '" + dest + "'. " + ex);
                        return;
                    }
                }

                int count = CopyDirectory(source, dest);
            }
            catch (Exception ex)
            {
                Debug.LogError("[OctNav] Post-build copy failed: " + ex);
            }
        }

        private static string GetBuildDirectory(BuildReport report)
        {
            string outputPath = report.summary.outputPath;
            BuildTarget target = report.summary.platform;

            if (target == BuildTarget.StandaloneOSX)
            {
                return Path.GetDirectoryName(outputPath);
            }

            if (Directory.Exists(outputPath))
            {
                return outputPath;
            }

            return Path.GetDirectoryName(outputPath);
        }

        private static int CopyDirectory(string source, string dest)
        {
            Directory.CreateDirectory(dest);

            string[] files = Directory.GetFiles(source, "*", SearchOption.AllDirectories);
            for (int i = 0; i < files.Length; i++)
            {
                string file = files[i];
                string rel = file.Substring(source.Length + 1);
                string target = Path.Combine(dest, rel);
                string targetDir = Path.GetDirectoryName(target);

                if (!Directory.Exists(targetDir))
                {
                    Directory.CreateDirectory(targetDir);
                }

                File.Copy(file, target, true);
            }

            return files.Length;
        }
    }
}
#endif
