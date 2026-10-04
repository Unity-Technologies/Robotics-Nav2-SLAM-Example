using System;
using Unity.Multiplayer.Center.Analytics;
using Unity.Multiplayer.Center.Common;
using Unity.Multiplayer.Center.Questionnaire;
using Unity.Multiplayer.Center.Recommendations;
using Unity.Multiplayer.Center.Window.UI;
using Unity.Multiplayer.Center.Window.UI.RecommendationView;
using UnityEditor;
using UnityEngine;
using UnityEngine.UIElements;

namespace Unity.Multiplayer.Center.Window
{
    internal class RecommendationTabView : ITabView
    {
        QuestionnaireView m_QuestionnaireView;
        RecommendationView m_RecommendationView;

        RecommendationViewBottomBar m_BottomBarView;

        bool m_IsVisible;
        bool m_ShouldRefresh = true;

        [SerializeField]
        PreReleaseHandling m_PreReleaseHandling = new();

        [field: SerializeField] // Marked as redundant by Rider, but it is not.
        public string Name { get; private set; }

        public VisualElement RootVisualElement { get; set; }
        public IMultiplayerCenterAnalytics MultiplayerCenterAnalytics { get; set; }

        // Access to QuestionnaireView for testing purposes
        internal QuestionnaireView QuestionnaireView => m_QuestionnaireView;

        public RecommendationTabView(string name = "Recommendation")
        {
            Name = name;
            m_PreReleaseHandling.OnAllChecksFinished += PatchData;
            m_PreReleaseHandling.CheckForUpdates();
            Undo.undoRedoPerformed += OnUndoRedoPerformed;
        }

        void OnUndoRedoPerformed()
        {
            UserChoicesObject.instance.Save();
            m_QuestionnaireView?.Refresh();
            UpdateRecommendation(keepSelection:true);
        }

        public void Clear()
        {
            Undo.undoRedoPerformed -= OnUndoRedoPerformed;
            m_RecommendationView?.Clear();
            m_QuestionnaireView?.Clear();
            RootVisualElement?.Clear();
            m_QuestionnaireView = null;
            m_RecommendationView = null;
        }

        public void SetVisible(bool visible)
        {
            RootVisualElement.style.display = visible ? DisplayStyle.Flex : DisplayStyle.None;
            m_IsVisible = visible;
        }

        public void Refresh()
        {
            Debug.Assert(MultiplayerCenterAnalytics != null, "MultiplayerCenterAnalytics != null");
            RefreshPreReleaseHandling();
            if (!m_ShouldRefresh && RootVisualElement.childCount > 0) return;
            CreateStandardView();
            m_ShouldRefresh = false;
        }

        void RefreshPreReleaseHandling()
        {
            if (!m_PreReleaseHandling.IsReady)
            {
                m_PreReleaseHandling.OnAllChecksFinished += PatchData;
                m_PreReleaseHandling.CheckForUpdates();
            }
            else
            {
                m_PreReleaseHandling.PatchRecommenderSystemData();
            }
        }

        void CreateStandardView()
        {
            Clear();
            Undo.undoRedoPerformed -= OnUndoRedoPerformed;
            Undo.undoRedoPerformed += OnUndoRedoPerformed;
            MigrateUserChoices();

            // We need this because Bottom bar is a part of the Recommendations Tab and it should always stay
            // at the bottom of the view. So we need to make sure that the root tab element is always 100% height.
            RootVisualElement.style.height = Length.Percent(100);

            var horizontalContainer = new TwoPaneSplitView(0, 250, TwoPaneSplitViewOrientation.Horizontal);
            horizontalContainer.AddToClassList(StyleClasses.MainSplitView);
            horizontalContainer.name = "recommendation-tab-split-view";

            // This is used to make sure the left side does not grow to 100% as this is what would happen by default.
            // It feels not 100% correct. But it seems to be the only way to match the height of the 2 sides with how
            // our views are build currently.
            horizontalContainer.contentContainer.style.position = Position.Relative;

            m_QuestionnaireView = new QuestionnaireView(QuestionnaireObject.instance.Questionnaire);
            m_QuestionnaireView.OnQuestionnaireDataChanged += HandleQuestionnaireDataChanged;
            m_QuestionnaireView.OnPresetSelected += OnPresetSelected;
            m_QuestionnaireView.Root.AddToClassList(StyleClasses.MainSplitViewLeft);
            horizontalContainer.Add(m_QuestionnaireView.Root);

            m_RecommendationView = new RecommendationView();
            m_RecommendationView.Root.AddToClassList(StyleClasses.MainSplitViewRight);
            horizontalContainer.Add(m_RecommendationView.Root);
            RootVisualElement.Add(horizontalContainer);

            m_BottomBarView = new RecommendationViewBottomBar(MultiplayerCenterAnalytics);
            m_RecommendationView.OnPackageSelectionChanged += 
                () => m_BottomBarView.UpdatePackagesToInstall(m_RecommendationView.CurrentRecommendation, m_RecommendationView.AllPackages);
            RootVisualElement.Add(m_BottomBarView);
            UpdateRecommendation(keepSelection: true);
            
            m_BottomBarView.SetInfoTextForCheckingPackages(!m_PreReleaseHandling.IsReady);
        }

        void HandleQuestionnaireDataChanged()
        {
            UpdateRecommendation(keepSelection: false);
        }

        static void MigrateUserChoices()
        {
            var questionnaire = QuestionnaireObject.instance.Questionnaire;
            var userChoices = UserChoicesObject.instance;

            // make sure the version of the questionnaire is the same as the one in the user choices.
            if (questionnaire.Version != userChoices.QuestionnaireVersion && userChoices.UserAnswers.Answers.Count > 0)
            {
                Logic.MigrateUserChoices(questionnaire, userChoices);
            }
        }

        void UpdateRecommendation(bool keepSelection)
        {
            var questionnaire = QuestionnaireObject.instance.Questionnaire;
            var userChoices = UserChoicesObject.instance;

            var errors = Logic.ValidateAnswers(questionnaire, userChoices.UserAnswers);
            foreach (var error in errors)
            {
                Debug.LogError(error);
            }

            var recommendation = userChoices.Preset == Preset.None? null
                : RecommenderSystem.GetRecommendation(questionnaire, userChoices.UserAnswers);
            m_PreReleaseHandling.PatchPackages(recommendation);
            if (keepSelection)
            {
                RecommendationUtils.ApplyPreviousSelection(recommendation, userChoices.SelectedSolutions);
            }
            else if (recommendation != null) // we only send the event if there is a recommendation and it is a new one
            {
                MultiplayerCenterAnalytics.SendRecommendationEvent(userChoices.UserAnswers, userChoices.Preset);
            }

            m_RecommendationView.UpdateRecommendation(recommendation, m_PreReleaseHandling);
            m_BottomBarView.UpdatePackagesToInstall(recommendation, m_RecommendationView.AllPackages);
        }

        void PatchData()
        {
            m_PreReleaseHandling.PatchRecommenderSystemData();
            m_PreReleaseHandling.OnAllChecksFinished -= PatchData;
            m_ShouldRefresh = true;
            if(m_IsVisible) 
                Refresh();
        }

        void OnPresetSelected(Preset preset)
        {
            var (resultAnswerData, recommendation) = Logic.ApplyPresetToAnswerData(
                UserChoicesObject.instance.UserAnswers, preset, QuestionnaireObject.instance.Questionnaire);

            UserChoicesObject.instance.UserAnswers = resultAnswerData;
            UserChoicesObject.instance.Save();

            if (recommendation != null)
                MultiplayerCenterAnalytics.SendRecommendationEvent(resultAnswerData, preset);

            m_QuestionnaireView.Refresh();
            m_RecommendationView.UpdateRecommendation(recommendation, m_PreReleaseHandling);
        }
    }
}
