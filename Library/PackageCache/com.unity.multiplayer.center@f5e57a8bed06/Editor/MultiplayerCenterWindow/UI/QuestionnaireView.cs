using System;
using System.Collections.Generic;
using Unity.Multiplayer.Center.Common;
using Unity.Multiplayer.Center.Questionnaire;
using Unity.Multiplayer.Center.Recommendations;
using UnityEngine;
using UnityEngine.UIElements;

namespace Unity.Multiplayer.Center.Window.UI
{
    internal class QuestionnaireView
    {
        public VisualElement Root { get; private set; }
        readonly QuestionnaireData m_Questions;

        public event Action OnQuestionnaireDataChanged;
        public event Action<Preset> OnPresetSelected;

        public QuestionnaireView(QuestionnaireData questions)
        {
            m_Questions = questions;
            Root = new VisualElement();
            Root.name = "questionnaire-view";
            Refresh();
        }

        public void Refresh()
        {
            RefreshData();
            ReCreateVisualElements();
        }

        void ReCreateVisualElements()
        {
            Root.Clear();

            var existingAnswers = UserChoicesObject.instance.UserAnswers.Answers;
            var questions = m_Questions.Questions;
            
            
            var gameSpecs = new QuestionSection(questions, existingAnswers, "Game Specifications", true);
            gameSpecs.AddPresetView();
            gameSpecs.OnPresetSelected += RaisePresetSelected;
            gameSpecs.QuestionUpdated += QuestionUpdated;
            Root.Add(gameSpecs);
            gameSpecs.CreateAdvancedFoldout(questions, existingAnswers, "Detailed Game Specifications");
            EvaluateAdvancedSectionVisibility();
        }

        void EvaluateAdvancedSectionVisibility()
        {
            var showAdvanced = UserChoicesObject.instance.Preset != Preset.None &&
                Logic.AreMandatoryQuestionsFilled(QuestionnaireObject.instance.Questionnaire, UserChoicesObject.instance.UserAnswers);
            
            Root.Q<QuestionSection>().SetAdvancedSectionVisible(showAdvanced);
        }

        public void Clear()
        {
            OnQuestionnaireDataChanged = null;
            Root.Clear();
        }

        static void RefreshData()
        {
            UserChoicesObject.instance.UserAnswers.Answers ??= new List<AnsweredQuestion>();
        }
        
        internal void QuestionUpdated(AnsweredQuestion answeredQuestion)
        {
            Logic.Update(UserChoicesObject.instance.UserAnswers, answeredQuestion);
            UserChoicesObject.instance.Save();
            if (IsAllQuestionsAnswered())
            {
                OnQuestionnaireDataChanged?.Invoke();
            }
            EvaluateAdvancedSectionVisibility();
        }
        
        internal void RaisePresetSelected(Preset preset)
        {
            OnPresetSelected?.Invoke(preset);
        }
        
        bool IsAllQuestionsAnswered()
        {
            foreach (var question in m_Questions.Questions)
            {
                if (!RecommendationUtils.IsQuestionAnswered(question))
                {
                    return false;
                }
            }

            return true;
        }
    }
}
