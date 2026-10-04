using NUnit.Framework;
using Unity.Multiplayer.Center.Analytics;
using Unity.Multiplayer.Center.Common;
using Unity.Multiplayer.Center.Common.Analytics;
using Unity.Multiplayer.Center.Window;
using UnityEngine;
using UnityEngine.UIElements;

namespace Unity.MultiplayerCenterTests
{
    [TestFixture]
    internal class RecommendationAnalyticsTests
    {
        [OneTimeSetUp]
        public void OneTimeSetup()  
        {
            // Copy user choices to temp file to restore after tests.
            UtilsForMultiplayerCenterTests.CopyUserChoicesToTempFile();
        }
        
        [OneTimeTearDown]
        public void OneTimeTearDown()
        {
            // restore user choices after tests.
            UtilsForMultiplayerCenterTests.RestoreUserChoicesFromTempFile();
        }

        [SetUp]
        public void SetUp()
        {
            UtilsForMultiplayerCenterTests.CloseMultiplayerCenterWindow();
        }
        
        [TearDown]
        public void TearDown()
        {
            UtilsForMultiplayerCenterTests.CloseMultiplayerCenterWindow();
        }

        class MockThatCountsEvents : IMultiplayerCenterAnalytics
        {
            public int InstallationEventCount { get; private set; }
            public int RecommendationEventCount { get; private set; }

            public void SendInstallationEvent(AnswerData data, Preset preset, Package[] packages,
                string hostingModelName, bool hostingModelRecommended, string netcodeSolutionName, bool netcodeSolutionRecommended)
                => InstallationEventCount++;

            public void SendRecommendationEvent(AnswerData data, Preset preset)
                => RecommendationEventCount++;
            
            public void SendGettingStartedInteractionEvent(string targetPackageId, string sectionId,
                InteractionDataType type, string displayName) { }
        }

        [Test]
        public void RecommendationTabView_PresetSelectedViaUI_RecommendationEventSent()
        {
            UtilsForMultiplayerCenterTests.PopulateUserAnswersForPresetAndPlayerCount(Preset.Async, 4);
            
            var view = CreateViewWithMockedAnalytics(out var mock);

            Assert.NotNull(view.QuestionnaireView);
            
            Assert.AreEqual(0, mock.RecommendationEventCount);
            
            view.QuestionnaireView.RaisePresetSelected(Preset.Shooter);
            Assert.AreEqual(1, mock.RecommendationEventCount);
        }

        [Test]
        public void RecommendationTabView_NonePresetSelected_RecommendationEventNotSent()
        {
            UtilsForMultiplayerCenterTests.PopulateUserAnswersForPresetAndPlayerCount(Preset.Async, 4);
            
            var view = CreateViewWithMockedAnalytics(out var mock);
            
            Assert.NotNull(view.QuestionnaireView);
            
            Assert.AreEqual(0, mock.RecommendationEventCount);
            
            view.QuestionnaireView.RaisePresetSelected(Preset.None);
            Assert.AreEqual(0, mock.RecommendationEventCount); // no recommendation => no event
        }

        [Test]
        public void RecommendationTabView_PlayerCountChangedViaUI_RecommendationEventSent()
        {
            UtilsForMultiplayerCenterTests.PopulateUserAnswersForPresetAndPlayerCount(Preset.Adventure, 8);
            
            var view = CreateViewWithMockedAnalytics(out var mock);
            
            Assert.NotNull(view.QuestionnaireView);
            
            Assert.AreEqual(0, mock.RecommendationEventCount);
            
            view.QuestionnaireView.QuestionUpdated(UtilsForMultiplayerCenterTests.CreatePlayerCountAnswer(2));
            Assert.AreEqual(1, mock.RecommendationEventCount);
        }
        
        [Test]
        public void RecommendationTabView_NonMandatoryAnswerChangedViaUI_RecommendationEventSent()
        {
            UtilsForMultiplayerCenterTests.PopulateUserAnswersForPresetAndPlayerCount(Preset.Adventure, 8);
            
            var view = CreateViewWithMockedAnalytics(out var mock);
            
            Assert.NotNull(view.QuestionnaireView);
            
            Assert.AreEqual(0, mock.RecommendationEventCount);
            
            view.QuestionnaireView.QuestionUpdated(UtilsForMultiplayerCenterTests.GetAnsweredQuestionThatIsNotInAdventurePreset());
            Assert.AreEqual(1, mock.RecommendationEventCount);
        }

        static RecommendationTabView CreateViewWithMockedAnalytics(out MockThatCountsEvents mock)
        {
            mock = new MockThatCountsEvents();
            var view = new RecommendationTabView();
            view.RootVisualElement = new VisualElement();
            view.MultiplayerCenterAnalytics = mock;
            view.Refresh();
            return view;
        }
    }
}
