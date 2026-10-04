using System;
using Unity.Multiplayer.Center.Analytics;
using UnityEditor;
using UnityEngine;
using UnityEngine.UIElements;

namespace Unity.Multiplayer.Center.Window
{
    // Note: there is a TabView API in UI Toolkit, but only starting from 2023.2
    internal interface ITabView
    {
        /// <summary>
        /// The name as displayed in the tab button
        /// Should be serialized.
        /// </summary>
        string Name { get; }

        /// <summary>
        /// The root visual element of the tab view.
        /// The setter will only be used if the root visual element is null when the tab is created.
        /// </summary>
        VisualElement RootVisualElement { get; set; }
        
        /// <summary>
        /// Sets the tab view visible or not.
        /// </summary>
        /// <param name="visible">If true, visible.</param>
        void SetVisible(bool visible);

        /// <summary>
        /// If true the Tab can be selected by the user.
        /// </summary>
        bool IsEnabled => true;

        /// <summary>
        /// Tooltip which will be shown on the Tab Button. 
        /// </summary>
        string ToolTip => "";

        /// <summary>
        /// Refreshes the UI Elements according to latest data.
        /// If the UI is not created yet, it does it.
        /// </summary>
        void Refresh();

        /// <summary>
        /// Unregister all events and clear UI Elements
        /// </summary>
        void Clear();
        
        /// <summary>
        /// The Multiplayer Center Analytics provider.
        /// </summary>
        IMultiplayerCenterAnalytics MultiplayerCenterAnalytics { get; set; }
    }

    [Serializable]
    internal class TabGroup
    {
        const string k_TabViewName = "tab-view";
        const string k_TabZoneName = "tab-zone";
        const string k_TabButtonUssClass = "tab-button";
        
        // The container for all the tabs
        const string k_TabsContainerUssClass ="tabs-container";
        
        // Gets applied to the root of each tab
        const string k_TabContentUssClass = "tab-content";

        [field: SerializeField]
        public int CurrentTab { get; private set; } = -1;

        public int ViewCount => m_TabViews?.Length ?? 0;

        VisualElement[] m_TabButtons;

        [SerializeReference]
        ITabView[] m_TabViews;

        public VisualElement Root { get; private set; }

        VisualElement m_MainContainer;

        IMultiplayerCenterAnalytics m_MultiplayerCenterAnalytics;
        
        internal IMultiplayerCenterAnalytics MultiplayerCenterAnalytics 
        {
            get => m_MultiplayerCenterAnalytics;
            set
            {
                m_MultiplayerCenterAnalytics = value;
                foreach (var tabView in m_TabViews)
                {
                    if(tabView != null)
                        tabView.MultiplayerCenterAnalytics = value;
                }
            }
        }

        public TabGroup(IMultiplayerCenterAnalytics analytics, ITabView[] tabViews, int defaultIndex = 0)
        {
            m_TabViews = tabViews;
            CurrentTab = defaultIndex;
            MultiplayerCenterAnalytics = analytics;
        }

        public void SetSelected(int index, bool force = false)
        {
            // Select the first tab, if the requested tab is not enabled.
            // This assumes the first tab is always enabled.
            if (!m_TabViews[index].IsEnabled)
                index = 0;

            if (index == CurrentTab && !force)
                return;

            if (CurrentTab >= 0 && CurrentTab < m_TabViews.Length)
            {
                m_TabButtons[CurrentTab].RemoveFromClassList("selected");
                m_TabViews[CurrentTab].SetVisible(false);
            }

            EditorPrefs.SetInt(PlayerSettings.productName + "_MultiplayerCenter_TabIndex", index);
            CurrentTab = index;
            m_TabViews[CurrentTab].Refresh();
            m_TabButtons[CurrentTab].AddToClassList("selected");
            m_TabViews[CurrentTab].SetVisible(true);
        }

        /// <summary>
        /// Instantiates the visual elements for all the tabs.
        /// Use this to create the tabs for the first time the UI is shown or after a domain reload.
        /// </summary>
        public void CreateTabs()
        {
            Root ??= new VisualElement();
            m_MainContainer ??= new VisualElement();

            if (Root.Q(k_TabZoneName) != null)
                Root.Q(k_TabZoneName).RemoveFromHierarchy();

            var tabZone = new VisualElement() {name = k_TabZoneName};
            Root.Add(tabZone);
            Root.name = k_TabViewName;
            m_TabButtons = new VisualElement[m_TabViews.Length];
            for (var i = 0; i < m_TabViews.Length; i++)
            {
                var tabView = m_TabViews[i];
                var index = i; // copy for closure
                var tabButton = new Button(() => SetSelected(index));
                tabButton.enabledSelf = tabView.IsEnabled;
                tabButton.tooltip = tabView.ToolTip;
                tabButton.AddToClassList(k_TabButtonUssClass);
                tabButton.text = tabView.Name;
                tabZone.Add(tabButton);
                m_TabButtons[i] = tabButton;
                tabView.RootVisualElement ??= new VisualElement();
                tabView.RootVisualElement.AddToClassList(k_TabContentUssClass);
                tabView.RootVisualElement.style.display = DisplayStyle.None;
                m_MainContainer.Add(m_TabViews[i].RootVisualElement);
            }

            m_MainContainer.AddToClassList(k_TabsContainerUssClass);
            Root.Add(m_MainContainer);
            CurrentTab = EditorPrefs.GetInt(PlayerSettings.productName + "_MultiplayerCenter_TabIndex", 0);
        }

        static void SetVisible(VisualElement e, bool visible)
        {
            e.style.display = visible ? DisplayStyle.Flex : DisplayStyle.None;
        }

        public void Clear()
        {
            if(m_TabViews == null)
                return;

            foreach (var tabView in m_TabViews)
            {
                tabView?.Clear();
            }
        }

        public bool TabsAreValid()
        {
            if (m_TabViews == null)
                return false;

            foreach (var tab in m_TabViews)
            {
                if (tab == null)
                    return false;
            }
            return true;
        }
    }
}
