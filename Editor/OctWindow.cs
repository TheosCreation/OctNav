using System.Collections.Generic;
using UnityEditor.UIElements;
using UnityEditor;
using UnityEngine;
using UnityEngine.UIElements;

namespace OctNav
{
    public class OctWindow : EditorWindow
    {
        private VisualElement buildPanel;
        private VisualElement gizmoPanel;
        private VisualElement settingsPanel;

        private OctVolume selectedVolume = null;
        private PopupField<OctVolume> volumePopup = null;

        [MenuItem("Tools/OctWindow %#o")]
        public static void ShowWindow()
        {
            OctWindow window = GetWindow<OctWindow>();
            window.titleContent = new GUIContent("OctNav");
            window.minSize = new Vector2(400, 500);
            window.Show();
        }

        private void OnEnable()
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

            #region Toolbar
            {
                VisualElement toolbar = new VisualElement
                {
                    style =
                    {
                        flexDirection = FlexDirection.Row,
                        justifyContent = Justify.SpaceAround,
                        marginBottom = 5
                    }
                };

                Button buildTab = new Button(() => ShowPanel("Build")) { text = "Build", style = { flexGrow = 1 } };
                Button gizmoTab = new Button(() => ShowPanel("Gizmos")) { text = "Gizmos", style = { flexGrow = 1 } };
                Button settingsTab = new Button(() => ShowPanel("Settings")) { text = "Settings", style = { flexGrow = 1 } };

                toolbar.Add(buildTab);
                toolbar.Add(gizmoTab);
                toolbar.Add(settingsTab);
                rootVisualElement.Add(toolbar);
            }
            #endregion

            #region Target-Volume Selector (always visible)
            {
                List<OctVolume> volumes = new List<OctVolume>(Resources.FindObjectsOfTypeAll<OctVolume>());
                if (volumes.Count > 0)
                {
                    volumePopup = new PopupField<OctVolume>(
                        "Target Volume",
                        volumes,
                        0,
                        vol => vol.gameObject.name
                    );
                    volumePopup.tooltip = "Select a specific OctVolume to target";
                    volumePopup.RegisterValueChangedCallback(evt =>
                    {
                        selectedVolume = evt.newValue;
                        RefreshAllPanels();
                    });

                    selectedVolume = volumes[0];
                    rootVisualElement.Add(volumePopup);
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

            #region Build Panel
            {
                buildPanel = new ScrollView { name = "Build", style = { flexGrow = 1 } };

                Button buildSelectedButton = new Button(() =>
                {
                    if (selectedVolume != null)
                    {
                        selectedVolume.BuildChildren();
                    }
                })
                { text = "Build Selected Volume" };
                buildPanel.Add(buildSelectedButton);

                Button resetSelectedButton = new Button(() =>
                {
                    if (selectedVolume != null)
                    {
                        selectedVolume.Reset();
                    }
                })
                { text = "Reset Selected Volume" };
                buildPanel.Add(resetSelectedButton);

                buildPanel.Add(new VisualElement() { style = { height = 10 } });

                Label globalLabel = new Label("Global Controls")
                {
                    style = { unityFontStyleAndWeight = FontStyle.Bold, fontSize = 12 }
                };
                buildPanel.Add(globalLabel);

                Button buildAllButton = new Button(() =>
                {
                    OctVolumeEditor.BuildAllVolumes();
                })
                { text = "Build All Octrees" };
                buildPanel.Add(buildAllButton);

                Button resetAllButton = new Button(() =>
                {
                    OctVolumeEditor.ResetAllVolumes();
                })
                { text = "Reset All Octrees" };
                buildPanel.Add(resetAllButton);

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
                    if (selectedVolume == null) return;
                    selectedVolume.showOctreeGizmos = evt.newValue;
                    EditorUtility.SetDirty(selectedVolume);
                    SceneView.RepaintAll();
                });
                gizmoPanel.Add(showOctreeToggle);

                Toggle showHitToggle = new Toggle("Show Collision Leafs")
                {
                    value = selectedVolume != null && selectedVolume.showHitGizmos
                };
                showHitToggle.RegisterValueChangedCallback(evt =>
                {
                    if (selectedVolume == null) return;
                    selectedVolume.showHitGizmos = evt.newValue;
                    EditorUtility.SetDirty(selectedVolume);
                    SceneView.RepaintAll();
                });
                gizmoPanel.Add(showHitToggle);

                Toggle drawAStarToggle = new Toggle("Draw A* Graph")
                {
                    value = selectedVolume != null && selectedVolume.drawAStarGizmos
                };
                drawAStarToggle.RegisterValueChangedCallback(evt =>
                {
                    if (selectedVolume == null) return;
                    selectedVolume.drawAStarGizmos = evt.newValue;
                    EditorUtility.SetDirty(selectedVolume);
                    SceneView.RepaintAll();
                });
                gizmoPanel.Add(drawAStarToggle);

                Toggle showSurfaceToggle = new Toggle("Show Surface Objects")
                {
                    value = selectedVolume != null && selectedVolume.showSurfaceObjectGizmos
                };
                showSurfaceToggle.RegisterValueChangedCallback(evt =>
                {
                    if (selectedVolume == null) return;
                    selectedVolume.showSurfaceObjectGizmos = evt.newValue;
                    EditorUtility.SetDirty(selectedVolume);
                    SceneView.RepaintAll();
                });
                gizmoPanel.Add(showSurfaceToggle);

                Toggle onlyWhenSelectedToggle = new Toggle("Only Draw Gizmos On Selected")
                {
                    value = selectedVolume != null && selectedVolume.drawGizmosOnlyWhenSelected
                };
                onlyWhenSelectedToggle.RegisterValueChangedCallback(evt =>
                {
                    if (selectedVolume == null) return;
                    selectedVolume.drawGizmosOnlyWhenSelected = evt.newValue;
                    EditorUtility.SetDirty(selectedVolume);
                    SceneView.RepaintAll();
                });
                gizmoPanel.Add(onlyWhenSelectedToggle);

                Toggle fadeToggle = new Toggle("Enable Distance Fade")
                {
                    value = selectedVolume != null && selectedVolume.enableDistanceFade
                };
                fadeToggle.tooltip = "Fade gizmos based on camera distance.";
                gizmoPanel.Add(fadeToggle);

                FloatField fadeStartField = new FloatField("Fade Start Distance")
                {
                    value = selectedVolume != null ? selectedVolume.fadeStartDistance : 0f
                };
                fadeStartField.style.marginLeft = 10;
                fadeStartField.style.display = (selectedVolume != null && selectedVolume.enableDistanceFade)
                    ? DisplayStyle.Flex
                    : DisplayStyle.None;
                fadeStartField.RegisterValueChangedCallback(evt =>
                {
                    if (selectedVolume == null) return;
                    selectedVolume.fadeStartDistance = evt.newValue;
                    EditorUtility.SetDirty(selectedVolume);
                    SceneView.RepaintAll();
                });
                gizmoPanel.Add(fadeStartField);

                FloatField fadeEndField = new FloatField("Fade End Distance")
                {
                    value = selectedVolume != null ? selectedVolume.fadeEndDistance : 0f
                };
                fadeEndField.style.marginLeft = 10;
                fadeEndField.style.display = (selectedVolume != null && selectedVolume.enableDistanceFade)
                    ? DisplayStyle.Flex
                    : DisplayStyle.None;
                fadeEndField.RegisterValueChangedCallback(evt =>
                {
                    if (selectedVolume == null) return;
                    selectedVolume.fadeEndDistance = evt.newValue;
                    EditorUtility.SetDirty(selectedVolume);
                    SceneView.RepaintAll();
                });
                gizmoPanel.Add(fadeEndField);

                fadeToggle.RegisterValueChangedCallback(evt =>
                {
                    if (selectedVolume == null) return;
                    selectedVolume.enableDistanceFade = evt.newValue;
                    EditorUtility.SetDirty(selectedVolume);

                    fadeStartField.style.display = evt.newValue ? DisplayStyle.Flex : DisplayStyle.None;
                    fadeEndField.style.display = evt.newValue ? DisplayStyle.Flex : DisplayStyle.None;

                    SceneView.RepaintAll();
                });

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
                indexSlider.style.display = (selectedVolume != null && selectedVolume.selectionMode)
                    ? DisplayStyle.Flex
                    : DisplayStyle.None;
                indexSlider.RegisterValueChangedCallback(evt =>
                {
                    if (selectedVolume == null) return;
                    selectedVolume.currentNodeIndex = Mathf.Max(0, evt.newValue);
                    EditorUtility.SetDirty(selectedVolume);
                    FocusOnSelectedNode();
                    SceneView.RepaintAll();
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
                previousButton.style.display = (selectedVolume != null && selectedVolume.selectionMode)
                    ? DisplayStyle.Flex
                    : DisplayStyle.None;
                gizmoPanel.Add(previousButton);

                Button nextButton = new Button(() =>
                {
                    if (selectedVolume == null) return;
                    int maxIndex = OctVolume.GetAllLeafNodesCount() - 1;
                    selectedVolume.currentNodeIndex = Mathf.Min(maxIndex, selectedVolume.currentNodeIndex + 1);
                    indexSlider.value = selectedVolume.currentNodeIndex;
                    EditorUtility.SetDirty(selectedVolume);
                    SceneView.RepaintAll();
                })
                { text = "Next Node" };
                nextButton.style.marginLeft = 10;
                nextButton.style.display = (selectedVolume != null && selectedVolume.selectionMode)
                    ? DisplayStyle.Flex
                    : DisplayStyle.None;
                gizmoPanel.Add(nextButton);

                Button frameButton = new Button(() => { FocusOnSelectedNode(); } )

                { text = "Frame Selected Node" };
                frameButton.style.marginLeft = 10;
                frameButton.style.display = (selectedVolume != null && selectedVolume.selectionMode)
                    ? DisplayStyle.Flex
                    : DisplayStyle.None;

                gizmoPanel.Add(frameButton);

                selectionToggle.RegisterValueChangedCallback(evt =>
                {
                    if (selectedVolume == null) return;
                    selectedVolume.selectionMode = evt.newValue;
                    EditorUtility.SetDirty(selectedVolume);

                    indexSlider.style.display = evt.newValue ? DisplayStyle.Flex : DisplayStyle.None;
                    previousButton.style.display = evt.newValue ? DisplayStyle.Flex : DisplayStyle.None;
                    nextButton.style.display = evt.newValue ? DisplayStyle.Flex : DisplayStyle.None;
                    frameButton.style.display = evt.newValue ? DisplayStyle.Flex : DisplayStyle.None;

                    SceneView.RepaintAll();
                });

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
                    if (selectedVolume == null) return;
                    selectedVolume.maxDepth = Mathf.Max(1, evt.newValue);
                    EditorUtility.SetDirty(selectedVolume);
                });
                settingsPanel.Add(depthField);

                LayerMaskField maskField = new LayerMaskField("Geometry Mask")
                {
                    value = selectedVolume != null ? selectedVolume.geometryMask : ~0
                };
                maskField.RegisterValueChangedCallback(evt =>
                {
                    if (selectedVolume == null) return;
                    selectedVolume.geometryMask = evt.newValue;
                    EditorUtility.SetDirty(selectedVolume);
                });
                settingsPanel.Add(maskField);

                rootVisualElement.Add(settingsPanel);
            }
            #endregion

            ShowPanel("Build");
        }

        private void ShowPanel(string panelName)
        {
            buildPanel.style.display = panelName == "Build" ? DisplayStyle.Flex : DisplayStyle.None;
            gizmoPanel.style.display = panelName == "Gizmos" ? DisplayStyle.Flex : DisplayStyle.None;
            settingsPanel.style.display = panelName == "Settings" ? DisplayStyle.Flex : DisplayStyle.None;
        }

        private void RefreshAllPanels()
        {
            ConstructUI();
        }

        private void FocusOnSelectedNode()
        {
            if (selectedVolume == null) return;

            List<OctNode> allLeaves = OctVolume.GetAllLeafNodesInScene();
            if (allLeaves == null || allLeaves.Count == 0) return;

            int idx = Mathf.Clamp(selectedVolume.currentNodeIndex, 0, allLeaves.Count - 1);
            OctNode selectedNode = allLeaves[idx];

            Bounds b = new Bounds(
                selectedNode.bounds.center,
                Vector3.one * selectedNode.bounds.size.magnitude * 0.5f
            );
            SceneView.lastActiveSceneView.Frame(b, false);
        }
    }
}
