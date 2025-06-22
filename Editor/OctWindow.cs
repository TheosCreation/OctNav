using System.Collections.Generic;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEditor.UIElements;

using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UIElements;
using System;
using System.Linq;

namespace OctNav
{
    public class OctWindow : EditorWindow
    {

        private VisualElement buildPanel;
        private VisualElement gizmoPanel;
        private VisualElement settingsPanel;

        private Toolbar toolbar;
        private ToolbarButton buildTabButton;
        private ToolbarButton gizmosTabButton;
        private ToolbarButton settingsTabButton;

        private List<OctVolume> prevVolumes = new List<OctVolume>();
        private OctVolume selectedVolume = null;
        private PopupField<string> volumePopup = null;
        private List<OctVolume> allVolumes = new List<OctVolume>();
        private bool autoSelectNearest = false;
        private string currentTab = "Build";
        [MenuItem("Window/OctWindow %#o")]
        public static void ShowWindow()
        {
            OctWindow window = GetWindow<OctWindow>();
            window.titleContent = new GUIContent("OctNav", Resources.Load<Texture2D>("OctNavIcon"));
            window.minSize = new Vector2(400, 500);
            window.Show();
        }

        private void OnEnable()
        {
            EditorApplication.hierarchyChanged += OnHierarchyChanged;
            SceneManager.sceneLoaded += OnSceneLoaded;
            ConstructUI();
        }

        private void OnDisable()
        {
            EditorApplication.hierarchyChanged -= OnHierarchyChanged;
            EditorSceneManager.sceneOpened -= OnSceneOpened;
            SceneManager.sceneLoaded -= OnSceneLoaded;
        }
        private void OnHierarchyChanged()
        {
          /*  List<OctVolume> now = FindObjectsByType<OctVolume>(FindObjectsSortMode.InstanceID).ToList();
            if (now.Count != prevVolumes.Count || !now.SequenceEqual(prevVolumes))
            {
                prevVolumes = now;
                ConstructUI();
            }*/
        }
        private void OnSceneLoaded(Scene scene, LoadSceneMode mode)
        {
            ConstructUI();
        }

        private void OnSceneOpened(Scene scene, OpenSceneMode mode)
        {
            ConstructUI();
        }

        private void ConstructUI()
        {
            rootVisualElement.Clear();
            rootVisualElement.style.paddingLeft = 10;
            rootVisualElement.style.paddingRight = 10;
            rootVisualElement.style.paddingTop = 10;
            rootVisualElement.style.flexDirection = FlexDirection.Column;
            allVolumes = new List<OctVolume>(FindObjectsByType<OctVolume>(FindObjectsSortMode.InstanceID));
            if (selectedVolume != null && !allVolumes.Contains(selectedVolume)) selectedVolume = null;

            #region Header
            {
                VisualElement headerContainer = new VisualElement
                {
                    style =
                    {
                        flexDirection = FlexDirection.Row,
                        justifyContent = Justify.Center,
                        alignItems = Align.Center,
                        marginTop = 5,
                        marginBottom = 5
                    }
                };
                Texture2D logoTex = Resources.Load<Texture2D>("OctNavLogo");

                if (logoTex != null)
                {
                    Image logoImage = new Image
                    {
                        image = logoTex,
                        scaleMode = ScaleMode.ScaleToFit,
                        style =
                        {
                            width = 320,
                            height = 320,
                            marginRight = 6
                        }
                    };
                    headerContainer.Add(logoImage);
                }
                else
                {
                    Debug.Log("Oh no");
                }

                rootVisualElement.Add(headerContainer);
            }
            #endregion

           

            #region Volume Selector
            {
                List<string> popupChoices = new List<string> { "All Volumes" };
                foreach (OctVolume vol in allVolumes)
                {
                    popupChoices.Add(vol.gameObject.name);
                }
                if (popupChoices.Count > 1)
                {
                    int initialIndex = 0;
                    if (selectedVolume != null)
                    {
                        string targetName = selectedVolume.gameObject.name;
                        int found = popupChoices.IndexOf(targetName);
                        if (found >= 0) initialIndex = found;
                    }
                    volumePopup = new PopupField<string>("Target Volume", popupChoices, initialIndex, str => str);

                    volumePopup.tooltip = "Select a volume to apply settings on or select all volumes";

                    volumePopup.RegisterValueChangedCallback(evt =>
                    {
                        string chosen = evt.newValue;
                        if (chosen == "All Volumes")
                        {
                            selectedVolume = null;
                        }
                        else
                        {
                            selectedVolume = allVolumes.Find(v => v.gameObject.name == chosen);
                        }
                        RefreshAllPanels();
                    });

                    // default to all
                    rootVisualElement.Add(volumePopup);
                    volumePopup.style.paddingBottom = 24;
                }
                else
                {
                    Label noVolumeLabel = new Label("No OctVolume found in scene");
                    noVolumeLabel.style.unityFontStyleAndWeight = FontStyle.Italic;
                    noVolumeLabel.style.marginBottom = 5;
                    rootVisualElement.Add(noVolumeLabel);
                    selectedVolume = null;
                }
            }
            #endregion

            #region Toolbar
            {
                toolbar = new Toolbar { style = { marginBottom = 8 } };

                buildTabButton = new ToolbarButton(() => ShowPanel("Build")) { text = "Build" };
                gizmosTabButton = new ToolbarButton(() => ShowPanel("Gizmos")) { text = "Gizmos" };
                settingsTabButton = new ToolbarButton(() => ShowPanel("Settings")) { text = "Settings" };

                buildTabButton.style.flexGrow = 1;
                gizmosTabButton.style.flexGrow = 1;
                settingsTabButton.style.flexGrow = 1;

                toolbar.Add(buildTabButton);
                toolbar.Add(gizmosTabButton);
                toolbar.Add(settingsTabButton);
                rootVisualElement.Add(toolbar);
                RefreshTabStyles();

            }
            #endregion

            #region Build Panel
            {
           
                buildPanel = new ScrollView { name = "Build", style = { flexGrow = 1 } };


                VisualElement warningBox = new VisualElement();
                warningBox.style.flexDirection = FlexDirection.Row;
                warningBox.style.alignItems = Align.Center;
                warningBox.style.marginTop = 10;
                warningBox.style.marginBottom = 10;

                Label warningLabel = new Label("OctNav recommends using convex shapes.\nNon-convex geometry near octree edges may cause mispathing.");
                warningLabel.style.color = Color.yellow;
                warningLabel.style.unityFontStyleAndWeight = FontStyle.Bold;
                warningLabel.style.whiteSpace = WhiteSpace.Normal;
                warningLabel.style.flexGrow = 1;
                warningBox.Add(warningLabel);

                Button closeButton = new Button(() =>
                {
                    warningBox.style.display = DisplayStyle.None;
                })
                { text = "x" };
                closeButton.style.marginLeft = 6;
                closeButton.tooltip = "Dismiss";
                warningBox.Add(closeButton);



                buildPanel.Add(warningBox);

                Button buildSelectedButton = new Button(() =>
                {
                    if (selectedVolume == null)
                    {
                        foreach (OctVolume vol in allVolumes)
                        {
                            vol.Build();
                        }
                    }
                    else
                    {
                        selectedVolume.Build();
                    }
                })
                { text = "Build Selected Volume" };
                buildPanel.Add(buildSelectedButton);


                Button resetSelectedButton = new Button(() =>
                {
                    if (selectedVolume == null)
                    {
                        foreach (OctVolume vol in allVolumes)
                        {
                            vol.ResetOctree();
                        }
                    }
                    else
                    {
                        selectedVolume.ResetOctree();
                    }
                })
                { text = "Reset Selected Volume" };
                buildPanel.Add(resetSelectedButton);


                buildPanel.Add(new VisualElement() { style = { height = 10 } });

                rootVisualElement.Add(buildPanel);
            }
            #endregion

            #region Gizmo Panel
            {
                gizmoPanel = new ScrollView { name = "Gizmos", style = { flexGrow = 1, display = DisplayStyle.None } };

                Label gizmoLabel = new Label("Gizmo Visibility")
                {
                    style = { unityFontStyleAndWeight = FontStyle.Bold, fontSize = 12 }
                };
                gizmoPanel.Add(gizmoLabel);


                Toggle showOctreeToggle = new Toggle("Show Octree")
                {
                    value = selectedVolume != null && selectedVolume.showOctreeGizmos
                };
                showOctreeToggle.RegisterValueChangedCallback(evt =>
                {
                    ApplyValue(vol => vol.showOctreeGizmos = evt.newValue);
                });
                gizmoPanel.Add(showOctreeToggle);

                Toggle showEmptyToggle = new Toggle("Show Empty Leaves")
                {
                    value = selectedVolume != null && selectedVolume.showEmptyLeavesGizmos
                };
                showEmptyToggle.RegisterValueChangedCallback(evt =>
                {
                    ApplyValue(vol => vol.showEmptyLeavesGizmos = evt.newValue);
                });
                gizmoPanel.Add(showEmptyToggle);


                Toggle showHitToggle = new Toggle("Show Collision Leaves")
                {
                    value = selectedVolume != null && selectedVolume.showHitGizmos
                };
                showHitToggle.RegisterValueChangedCallback(evt =>
                {
                     ApplyValue(vol => vol.showHitGizmos = evt.newValue);
                });
                gizmoPanel.Add(showHitToggle);


                Toggle drawAStarToggle = new Toggle("Draw A* Graph")
                {
                    value = selectedVolume != null && selectedVolume.drawGraph
                };
                drawAStarToggle.RegisterValueChangedCallback(evt =>
                {
                    ApplyValue(vol => vol.drawGraph = evt.newValue);
                });
                gizmoPanel.Add(drawAStarToggle);


                Toggle drawGroundedAStarToggle = new Toggle("Draw Grounded A* Graph")
                {
                    value = selectedVolume != null && selectedVolume.drawGroundedGraph
                };
                drawGroundedAStarToggle.RegisterValueChangedCallback(evt =>
                {
                    ApplyValue(vol => vol.drawGroundedGraph = evt.newValue);
                });
                gizmoPanel.Add(drawGroundedAStarToggle);


                Toggle showSurfaceToggle = new Toggle("Show Surface Objects")
                {
                    value = selectedVolume != null && selectedVolume.showSurfaceObjectGizmos
                };
                showSurfaceToggle.RegisterValueChangedCallback(evt =>
                {
                    ApplyValue(vol => vol.showSurfaceObjectGizmos = evt.newValue);
                });
                gizmoPanel.Add(showSurfaceToggle);

                //will be updated to work with oct window selected voulme

                /*Toggle onlyWhenSelectedToggle = new Toggle("Only Draw Gizmos On Selected Volumes")
                {
                    value = selectedVolume != null && selectedVolume.drawGizmosOnlyWhenSelected
                };
                onlyWhenSelectedToggle.RegisterValueChangedCallback(evt =>
                {
                    ApplyValue(vol => vol.drawGizmosOnlyWhenSelected = evt.newValue);
                });
                gizmoPanel.Add(onlyWhenSelectedToggle);*/


                Toggle fadeToggle = new Toggle("Enable Distance Fade")
                {
                    value = selectedVolume != null && selectedVolume.enableDistanceFade
                };
                fadeToggle.tooltip = "Fade gizmos based on camera distance";
                gizmoPanel.Add(fadeToggle);


                FloatField fadeStartField = new FloatField("Fade Start Distance")
                {
                    value = selectedVolume != null ? selectedVolume.fadeStartDistance : 0f
                };
                fadeStartField.style.marginLeft = 10;
                fadeStartField.style.display = (selectedVolume != null && selectedVolume.enableDistanceFade) ? DisplayStyle.Flex : DisplayStyle.None;
                fadeStartField.RegisterValueChangedCallback(evt =>
                {
                    ApplyValue(vol => vol.fadeStartDistance = evt.newValue);
                });
                gizmoPanel.Add(fadeStartField);


                FloatField fadeEndField = new FloatField("Fade End Distance")
                {
                    value = selectedVolume != null ? selectedVolume.fadeEndDistance : 0f
                };
                fadeEndField.style.marginLeft = 10;
                fadeEndField.style.display = (selectedVolume != null && selectedVolume.enableDistanceFade) ? DisplayStyle.Flex : DisplayStyle.None;
                fadeEndField.RegisterValueChangedCallback(evt =>
                {
                    ApplyValue(vol => vol.fadeEndDistance = evt.newValue);
                });
                gizmoPanel.Add(fadeEndField);


                fadeToggle.RegisterValueChangedCallback(evt =>
                {
                    ApplyValue(vol => vol.enableDistanceFade = evt.newValue, false);

                    fadeStartField.style.display = evt.newValue ? DisplayStyle.Flex : DisplayStyle.None;
                    fadeEndField.style.display = evt.newValue ? DisplayStyle.Flex : DisplayStyle.None;

                    SceneView.RepaintAll();
                });
                if (selectedVolume != null)
                {
                    Toggle selectionToggle = new Toggle("Enable Selection Mode")
                    {
                        value = selectedVolume != null && selectedVolume.selectionMode
                    };
                    selectionToggle.tooltip = "Toggle stepping through leaf nodes by index.";
                    gizmoPanel.Add(selectionToggle);


                    IntegerField indexSlider = new IntegerField("Node Index")
                    {
                        value = selectedVolume != null ? selectedVolume.currentNodeIndex : 0
                    };
                    indexSlider.style.marginLeft = 10;
                    indexSlider.style.display = (selectedVolume != null && selectedVolume.selectionMode) ? DisplayStyle.Flex : DisplayStyle.None;
                    indexSlider.RegisterValueChangedCallback(evt =>
                    {
                        ApplyValue(vol => vol.currentNodeIndex = Mathf.Max(0, evt.newValue));
                    });
                    gizmoPanel.Add(indexSlider);


                    Button previousButton = new Button(() =>
                    {
                        if (selectedVolume == null) return;
                        selectedVolume.currentNodeIndex = Mathf.Max(0, selectedVolume.currentNodeIndex - 1);
                        indexSlider.value = selectedVolume.currentNodeIndex;
                        EditorUtility.SetDirty(selectedVolume);
                        SceneView.RepaintAll();
                    })
                    { text = "Previous Node" };
                    previousButton.style.marginLeft = 10;
                    previousButton.style.display = (selectedVolume != null && selectedVolume.selectionMode) ? DisplayStyle.Flex : DisplayStyle.None;
                    gizmoPanel.Add(previousButton);


                    Button nextButton = new Button(() =>
                    {
                        if (selectedVolume == null) return;
                        int maxIndex = OctManager.GetAllLeafNodesCount() - 1;
                        selectedVolume.currentNodeIndex = Mathf.Min(maxIndex, selectedVolume.currentNodeIndex + 1);
                        indexSlider.value = selectedVolume.currentNodeIndex;
                        EditorUtility.SetDirty(selectedVolume);
                        SceneView.RepaintAll();
                    })
                    { text = "Next Node" };

                    nextButton.style.marginLeft = 10;
                    nextButton.style.display = (selectedVolume != null && selectedVolume.selectionMode) ? DisplayStyle.Flex : DisplayStyle.None;
                    gizmoPanel.Add(nextButton);


                    Button frameButton = new Button(() => { FocusOnSelectedNode(); })

                    { text = "Frame Selected Node" };
                    frameButton.style.marginLeft = 10;
                    frameButton.style.display = (selectedVolume != null && selectedVolume.selectionMode) ? DisplayStyle.Flex : DisplayStyle.None;

                    gizmoPanel.Add(frameButton);

                    Button selectNearestButton = new Button(() =>
                    {
                        if (selectedVolume == null) return;
                        SceneView sv = SceneView.lastActiveSceneView;
                        if (sv?.camera == null) return;

                        Vector3 camPos = sv.camera.transform.position;
                        List<OctNode> nodes = selectedVolume.allNodes;
                        if (nodes == null || nodes.Count == 0) return;

                        float bestSqr = float.MaxValue;
                        int bestIndex = 0;
                        for (int i = 0; i < nodes.Count; i++)
                        {
                            float dist = (nodes[i].bounds.center - camPos).sqrMagnitude;
                            if (dist < bestSqr) { bestSqr = dist; bestIndex = i; }
                        }

                        selectedVolume.currentNodeIndex = bestIndex;
                        indexSlider.value = bestIndex;
                        EditorUtility.SetDirty(selectedVolume);
                        sv.Repaint();
                    })
                    { text = "Select Nearest Node" };
                    selectNearestButton.style.marginLeft = 10;
                    selectNearestButton.style.display = (selectedVolume != null && selectedVolume.selectionMode)
                                                        ? DisplayStyle.Flex
                                                        : DisplayStyle.None;
                    selectNearestButton.tooltip = "Pick closest leaf to scene camera";
                    gizmoPanel.Add(selectNearestButton);

                    selectionToggle.RegisterValueChangedCallback(evt =>
                    {
                        ApplyValue(vol => vol.selectionMode = evt.newValue, false);

                        indexSlider.style.display = evt.newValue ? DisplayStyle.Flex : DisplayStyle.None;
                        previousButton.style.display = evt.newValue ? DisplayStyle.Flex : DisplayStyle.None;
                        nextButton.style.display = evt.newValue ? DisplayStyle.Flex : DisplayStyle.None;
                        frameButton.style.display = evt.newValue ? DisplayStyle.Flex : DisplayStyle.None;

                        SceneView.RepaintAll();
                    });
                }
                rootVisualElement.Add(gizmoPanel);
            }
            #endregion

            #region Settings Panel
            {
                settingsPanel = new ScrollView { name = "Settings", style = { flexGrow = 1, display = DisplayStyle.None } };

                Label settingsLabel = new Label("OctNav Settings")
                {
                    style = { unityFontStyleAndWeight = FontStyle.Bold, fontSize = 12 }
                };
                settingsPanel.Add(settingsLabel);


                IntegerField depthField = new IntegerField("Max Depth")
                {
                    value = selectedVolume != null ? selectedVolume.maxDepth : 1
                };
                depthField.RegisterValueChangedCallback(evt =>
                {
                    ApplyValue(vol => vol.maxDepth = Mathf.Max(1, evt.newValue));
                });
                settingsPanel.Add(depthField);


                LayerMaskField maskField = new LayerMaskField("Geometry Mask")
                {
                    value = selectedVolume != null ? selectedVolume.geometryMask : ~0
                };
                maskField.RegisterValueChangedCallback(evt =>
                {
                    ApplyValue(vol => vol.geometryMask = evt.newValue);
                });
                settingsPanel.Add(maskField);

                rootVisualElement.Add(settingsPanel);
            }
            #endregion

            ShowPanel(currentTab);
        }

        private void ShowPanel(string panelName)
        {
            currentTab = panelName;
            buildPanel.style.display = panelName == "Build" ? DisplayStyle.Flex : DisplayStyle.None;
            gizmoPanel.style.display = panelName == "Gizmos" ? DisplayStyle.Flex : DisplayStyle.None;
            settingsPanel.style.display = panelName == "Settings" ? DisplayStyle.Flex : DisplayStyle.None;
            RefreshTabStyles();
        }
        private void RefreshTabStyles()
        {
            Color selectedBg = new Color(0.275f, 0.376f, 0.486f, 1f);
            Color normalBg = new Color(0.345f, 0.345f, 0.345f, 1f);

            Color selectedTextColor = new Color(1f, 1f, 1f, 1f);
            Color normalTextColor = new Color(0.8f, 0.8f, 0.8f, 1f);

            if (currentTab == "Build")
            {
                buildTabButton.style.backgroundColor = new StyleColor(selectedBg);
                buildTabButton.style.color = new StyleColor(selectedTextColor);
            }
            else
            {
                buildTabButton.style.backgroundColor = new StyleColor(normalBg);
                buildTabButton.style.color = new StyleColor(normalTextColor);
            }

            if (currentTab == "Gizmos")
            {
                gizmosTabButton.style.backgroundColor = new StyleColor(selectedBg);
                gizmosTabButton.style.color = new StyleColor(selectedTextColor);
            }
            else
            {
                gizmosTabButton.style.backgroundColor = new StyleColor(normalBg);
                gizmosTabButton.style.color = new StyleColor(normalTextColor);
            }

            if (currentTab == "Settings")
            {
                settingsTabButton.style.backgroundColor = new StyleColor(selectedBg);
                settingsTabButton.style.color = new StyleColor(selectedTextColor);
            }
            else
            {
                settingsTabButton.style.backgroundColor = new StyleColor(normalBg);
                settingsTabButton.style.color = new StyleColor(normalTextColor);
            }
        }

        private void RefreshAllPanels()
        {
            ConstructUI();
        }
        private void ApplyValue(System.Action<OctVolume> action, bool repaint = true)
        {
            if (selectedVolume == null)
            {
                foreach (OctVolume vol in allVolumes)
                {
                    action(vol);
                    EditorUtility.SetDirty(vol);
                }
            }
            else
            {
                action(selectedVolume);
                EditorUtility.SetDirty(selectedVolume);
            }
            if (repaint)
            {
                SceneView.RepaintAll();
            }
        }
        private void FocusOnSelectedNode()
        {
            if (selectedVolume == null) return;

            List<OctNode> allLeaves = OctManager.GetAllLeafNodesInScene();

            if (allLeaves == null || allLeaves.Count == 0) return;

            int nodeIndex = Mathf.Clamp(selectedVolume.currentNodeIndex, 0, allLeaves.Count - 1);
            OctNode selectedNode = allLeaves[nodeIndex];

            Bounds nodeBounds = new Bounds(
                selectedNode.bounds.center,
                Vector3.one * selectedNode.bounds.size.magnitude * 0.5f
            );
            SceneView.lastActiveSceneView.Frame(nodeBounds, false);
        }
    }
}
