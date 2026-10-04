using NUnit.Framework;
using Unity.Multiplayer.Center.Analytics;
using Unity.Multiplayer.Center.Common;
using Unity.Multiplayer.Center.Common.Analytics;
using UnityEngine;
using Unity.Multiplayer.Center.Window;
using UnityEditor;
using UnityEngine.UIElements;

namespace Unity.MultiplayerCenterTests
{
    [TestFixture]
    class TabGroupTests
    {
        TabGroup m_TabGroup;
        RecommendationTabView m_RecommendationTabView;
        GettingStartedTabView m_GettingStartedTabView;

        private class AnalyticsMock : IMultiplayerCenterAnalytics
        {
            public void SendInstallationEvent(AnswerData data, Preset preset, Package[] packages,
                string hostingModelName, bool hostingModelRecommended, string netcodeSolutionName, bool netcodeSolutionRecommended) {}

            public void SendRecommendationEvent(AnswerData data, Preset preset){}

            public void SendGettingStartedInteractionEvent(string targetPackageId, string sectionId, InteractionDataType type, string displayName) {}
        }

        private class MockTabEnabled : ITabView
        {
            public string Name => "MockTabEnabled";
            public VisualElement RootVisualElement { get; set; }
            public void SetVisible(bool visible)
            {
                RootVisualElement.style.display = visible ? DisplayStyle.Flex : DisplayStyle.None;
            }
            
            public void Refresh() { }

            public void Clear() { }

            public IMultiplayerCenterAnalytics MultiplayerCenterAnalytics { get; set; }
        }
        
        private class MockTabDisabled : ITabView
        {
            public string Name => "MockTabDisbled";
            public VisualElement RootVisualElement { get; set; }
            public void SetVisible(bool visible)
            {
                RootVisualElement.style.display = visible ? DisplayStyle.Flex : DisplayStyle.None;
            }

            public bool IsEnabled => false;

            public void Refresh() { }

            public void Clear() { }

            public IMultiplayerCenterAnalytics MultiplayerCenterAnalytics { get; set; }
        }

        [SetUp]
        public void SetUp()
        {
            m_RecommendationTabView = new RecommendationTabView();
            m_GettingStartedTabView = new GettingStartedTabView();
            m_TabGroup = new TabGroup(new AnalyticsMock(), new ITabView[] {m_RecommendationTabView, m_GettingStartedTabView, new MockTabEnabled(), new MockTabDisabled()});
        }

        [Test]
        public void TabGroup_CreateTabs_4TabViews()
        {
            m_TabGroup.CreateTabs();
            Assert.AreEqual(4, m_TabGroup.ViewCount);
        }

        [Test]
        public void TabGroup_CreateTabs_SelectsTabFromUserPreferences()
        {
            m_TabGroup.CreateTabs();
            var currentTabFromEditorPrefs = EditorPrefs.GetInt(PlayerSettings.productName + "_MultiplayerCenter_TabIndex", 0);
            Assert.AreEqual(currentTabFromEditorPrefs, m_TabGroup.CurrentTab);
        }
        
        [Test]
        public void TabGroup_SelectDeactivatedTab_SelectsFirstTab()
        {
            m_TabGroup.CreateTabs();
            m_TabGroup.SetSelected(3);
            Assert.AreEqual(0, m_TabGroup.CurrentTab);
        }
        
        [Test]
        public void TabGroup_AnalyticsIsPropagatedToAllViews()
        {
            Assert.NotNull(m_RecommendationTabView.MultiplayerCenterAnalytics);
            Assert.NotNull(m_GettingStartedTabView.MultiplayerCenterAnalytics);
            Assert.AreEqual(m_RecommendationTabView.MultiplayerCenterAnalytics, m_GettingStartedTabView.MultiplayerCenterAnalytics);
            Assert.AreEqual(m_RecommendationTabView.MultiplayerCenterAnalytics.GetType(), typeof(AnalyticsMock));
        }
        
        [TearDown]
        public void TearDown()
        {
            if (m_TabGroup != null)
            {
                m_TabGroup.Clear();
            }
        }
    }
}
