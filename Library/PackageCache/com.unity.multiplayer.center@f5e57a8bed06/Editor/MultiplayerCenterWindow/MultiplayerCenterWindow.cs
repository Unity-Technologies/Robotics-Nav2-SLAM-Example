using System;
using Unity.Multiplayer.Center.Analytics;
using UnityEditor;
using UnityEngine;
using UnityEngine.UIElements;

namespace Unity.Multiplayer.Center.Window
{
    internal class MultiplayerCenterWindow : EditorWindow, ISerializationCallbackReceiver
    {
        const string k_PathInPackage = "Packages/com.unity.multiplayer.center/Editor/MultiplayerCenterWindow";
        const string k_SpinnerClassName = "processing";
        const string k_SessionStateDomainReloadKey = "MultiplayerCenter.InDomainReload";

        VisualElement m_SpinningIcon;

        /// <summary>
        /// Nest the main container in a VisualElement to allow for easy enabling/disabling of the entire window but
        /// without the spinning icon.
        /// </summary>
        VisualElement m_MainContainer;

        Vector2 m_WindowSize = new(350, 300);

        public int CurrentTab => m_TabGroup.CurrentTab;

        // Testing purposes only. We don't want to set CurrentTab from window
        internal int CurrentTabTest
        {
            get => m_TabGroup.CurrentTab;
            set => m_TabGroup.SetSelected(value);
        }

        [SerializeField]
        bool m_RequestGettingStartedTabAfterDomainReload = false;

        [SerializeField]
        TabGroup m_TabGroup;

        /// <summary>
        /// This is the reference Multiplayer Center analytics implementation. This class owns it.
        /// </summary>
        IMultiplayerCenterAnalytics m_MultiplayerCenterAnalytics;

        IMultiplayerCenterAnalytics MultiplayerCenterAnalytics => m_MultiplayerCenterAnalytics ??= MultiplayerCenterAnalyticsFactory.Create();

        [MenuItem("Window/Multiplayer/Multiplayer Center")]
        public static void OpenWindow()
        {
            var showUtility = false; // TODO: figure out if it would be a good idea to have a utility window (always on top, cannot be tabbed)
            GetWindow<MultiplayerCenterWindow>(showUtility, "Multiplayer Center", true);
        }

        void OnEnable()
        {
            // Adjust window size based on dpi scaling
            var dpiScale = EditorGUIUtility.pixelsPerPoint;
            minSize = new Vector2(m_WindowSize.x * dpiScale, m_WindowSize.y * dpiScale);

            AssemblyReloadEvents.beforeAssemblyReload -= OnBeforeDomainReload;
            AssemblyReloadEvents.afterAssemblyReload -= OnAfterDomainReload;
            AssemblyReloadEvents.beforeAssemblyReload += OnBeforeDomainReload;
            AssemblyReloadEvents.afterAssemblyReload += OnAfterDomainReload;
        }

        void OnDisable()
        {
            AssemblyReloadEvents.beforeAssemblyReload -= OnBeforeDomainReload;
            AssemblyReloadEvents.afterAssemblyReload -= OnAfterDomainReload;
        }

        /// <summary>
        /// Changes Tab from Recommendation to the Quickstart tab.
        /// </summary>
        public void RequestShowGettingStartedTabAfterDomainReload()
        {
            m_RequestGettingStartedTabAfterDomainReload = true;

            // If no domain reload is necessary, this will be called.
            // If domain reload is necessary, the delay call will be forgotten, but CreateGUI will be called like after any domain reload
            // An extra delay is added to make sure that the visibility conditions of the Quickstart tab have been
            // fully evaluated. This solves MTT-8939.
            EditorApplication.delayCall += () =>
            {
                rootVisualElement.schedule.Execute(CallCreateGuiWithQuickstartRequest).ExecuteLater(300);
            };
        }

        internal void DisableUiForInstallation()
        {
            SetSpinnerIconRotating();
            m_MainContainer.SetEnabled(false);
        }

        internal void ReenableUiAfterInstallation()
        {
            RemoveSpinnerIconRotating();
            m_MainContainer.SetEnabled(true);
        }

        void Update()
        {
            // Restore the GUI if it was cleared in OnBeforeSerialize.
            if (m_TabGroup == null || m_TabGroup.ViewCount < 1)
            {
                CreateGUI();
            }
        }

        void CreateGUI()
        {
            rootVisualElement.name = "root";
            m_MainContainer ??= new VisualElement();
            m_MainContainer.name = "recommendation-tab-container";
            m_MainContainer.Clear();
            rootVisualElement.Add(m_MainContainer);
            m_SpinningIcon = new VisualElement();
            var theme = EditorGUIUtility.isProSkin ? "dark" : "light";
            rootVisualElement.styleSheets.Add(AssetDatabase.LoadAssetAtPath<StyleSheet>($"{k_PathInPackage}/UI/{theme}.uss"));
            rootVisualElement.styleSheets.Add(AssetDatabase.LoadAssetAtPath<StyleSheet>($"{k_PathInPackage}/UI/MultiplayerCenterWindow.uss"));

            if (m_TabGroup == null || m_TabGroup.ViewCount < 1 || !m_TabGroup.TabsAreValid())
                m_TabGroup = new TabGroup(MultiplayerCenterAnalytics, new ITabView[] {new RecommendationTabView(), new GettingStartedTabView()});
            else // since we are not serializing the analytics provider, we need to set it again
                m_TabGroup.MultiplayerCenterAnalytics = MultiplayerCenterAnalytics;

            m_TabGroup.CreateTabs();
            m_MainContainer.Add(m_TabGroup.Root);

            var installationInProgress = !PackageManagement.IsInstallationFinished();
            SetWindowContentEnabled(installationInProgress, m_RequestGettingStartedTabAfterDomainReload);
            ShowAppropriateTab(installationInProgress);
        }

        void ShowAppropriateTab(bool installationInProgress)
        {
            if (installationInProgress)
            {
                PackageManagement.RegisterToExistingInstaller(b => RequestShowGettingStartedTabAfterDomainReload());
                m_TabGroup.SetSelected(0, force: true);
                return;
            }

            if (m_RequestGettingStartedTabAfterDomainReload)
            {
                m_RequestGettingStartedTabAfterDomainReload = false;
                m_TabGroup.SetSelected(1, force: true);
            }
            else
            {
                m_TabGroup.SetSelected(m_TabGroup.CurrentTab, force: true);
            }
        }

        void SetWindowContentEnabled(bool installationInProgress, bool quickstartRequested)
        {
            m_MainContainer.SetEnabled(!installationInProgress || quickstartRequested);

            // if we are current already processing an installation, show the spinning icon
            if (installationInProgress)
            {
                // Wait a bit because the animation does not trigger when we call this in CreateGUI
                EditorApplication.delayCall += SetSpinnerIconRotating;
            }

            rootVisualElement.Add(m_SpinningIcon);
        }

        void CallCreateGuiWithQuickstartRequest()
        {
            // Interestingly, setting this before registering the delay call sometimes results in the value
            // being false when CreateGUI starts, so we set it again here.
            m_RequestGettingStartedTabAfterDomainReload = true;
            CreateGUI();
        }

        void SetSpinnerIconRotating()
        {
            m_SpinningIcon.AddToClassList(k_SpinnerClassName);
        }

        void RemoveSpinnerIconRotating()
        {
            m_SpinningIcon?.RemoveFromClassList(k_SpinnerClassName);
        }

        void ClearTabs()
        {
            m_TabGroup?.Clear();
            m_TabGroup = null;
        }

        // This will not get called when the Editor is closed.
        void OnDestroy()
        {
            ClearTabs();
        }

        static void OnBeforeDomainReload()
        {
            SessionState.SetBool(k_SessionStateDomainReloadKey, true);
        }

        static void OnAfterDomainReload()
        {
            SessionState.SetBool(k_SessionStateDomainReloadKey, false);
        }

        public void OnBeforeSerialize()
        {
            // ClearTabs if the Window gets serialized, but we are not in DomainReload
            // This happens when the Editor closes or the WindowLayout is saved by the user.
            // This ensures that the State of the Tabs is not serialized into the WindowLayout of the User.
            if (SessionState.GetBool(k_SessionStateDomainReloadKey, false) == false)
            {
                ClearTabs();
            }
        }

        public void OnAfterDeserialize()
        {
            // Empty on purpose.
        }
    }
}
