using System;
using System.Collections.Generic;
using Unity.Multiplayer.Center.Common;
using Unity.Multiplayer.Center.Questionnaire;
using Unity.Multiplayer.Center.Recommendations;
using UnityEditor;
using UnityEditor.UIElements;
using UnityEngine;
using UnityEngine.UIElements;

namespace Unity.Multiplayer.Center.Window.UI
{
    /// <summary>
    /// Questions that go together in the Questionnaire view.
    /// Example: mandatory questions in "Game Specs" section, optional questions in "Advanced" section.
    /// </summary>
    internal class QuestionSection : VisualElement
    {
        EnumField m_PresetDropdown;
        public VisualElement ContentRoot { get; private set; }
        public event Action<AnsweredQuestion> QuestionUpdated;

        public event Action<Preset> OnPresetSelected;

        const string k_AdvancedFoldoutName = "advanced-questions";

        public void CreateAdvancedFoldout(Question[] questions, IEnumerable<AnsweredQuestion> existingAnswers, string headerTitle)
        {
            var foldout = new Foldout
            {
                text = headerTitle,
                name = k_AdvancedFoldoutName
            };
            foreach (var question in questions)
            {
                if (question.IsMandatory)
                    continue;

                foldout.Add(CreateSingleQuestionView(question, existingAnswers));
            }

            ContentRoot.Insert(1, foldout);
        }

        public QuestionSection(Question[] questions, IEnumerable<AnsweredQuestion> existingAnswers, string headerTitle,
            bool mandatoryQuestions)
        {
            var title = new VisualElement();
            title.Add(new Label() {text = headerTitle});
            title.AddToClassList(StyleClasses.ViewHeadline);
            Add(title);
            
            ContentRoot = CreateContentRoot(true);

            Add(ContentRoot);

            for (var index = 0; index < questions.Length; index++)
            {
                if ((!mandatoryQuestions && questions[index].IsMandatory) || (mandatoryQuestions && !questions[index].IsMandatory))
                    continue;

                ContentRoot.Add(CreateSingleQuestionView(questions[index], existingAnswers));
            }

            Undo.undoRedoPerformed += OnUndoRedoPerformed;
        }

        ~QuestionSection()
        {
            Undo.undoRedoPerformed -= OnUndoRedoPerformed;
        }

        void OnUndoRedoPerformed()
        {
            m_PresetDropdown?.SetValueWithoutNotify(UserChoicesObject.instance.Preset);
        }

        public void AddPresetView()
        {
            ContentRoot.Insert(0, CreatePresetView());
        }

        static VisualElement CreateContentRoot(bool withScrollView)
        {
            if (!withScrollView)
                return new VisualElement();

            var root = new ScrollView();
            root.horizontalScrollerVisibility = ScrollerVisibility.Hidden;
            return root;
        }

        VisualElement CreateSingleQuestionView(Question question, IEnumerable<AnsweredQuestion> existingAnswers)
        {
            Logic.TryGetAnswerByQuestionId(existingAnswers, question.Id, out var existingAnswer);
            existingAnswer ??= new AnsweredQuestion { QuestionId = question.Id };
            
            var questionView = GetQuestionHeader(question, out var questionContainer);
            questionContainer.Add(QuestionViewFactory.CreateQuestionViewInput(question, existingAnswer, RaiseQuestionUpdated));
            return questionView;
        }

        static VisualElement GetQuestionHeader(Question question, out VisualElement inputContainer)
        {
            return QuestionViewFactory.StandardQuestionHeader(question, out inputContainer);
        }

        void RaiseQuestionUpdated(AnsweredQuestion question)
        {
            QuestionUpdated?.Invoke(question);
        }

        VisualElement CreatePresetView()
        {
            const string description = "Select the game genre that is the closest match to your project to generate common game specifications for this genre and recommended solutions. Recommendations are based solely on player count and game specifications.";
            var presetQuestion = new Question()
                {Title = "Genre of your Game", Description = description, IsMandatory = true};

            var questionView = GetQuestionHeader(presetQuestion, out var questionContainer);
            questionContainer.Add(PresetDropdown());
            return questionView;
        }

        EnumField PresetDropdown()
        {
            m_PresetDropdown = new EnumField(UserChoicesObject.instance.Preset);
            m_PresetDropdown.RegisterValueChangedCallback(RaiseValueChangedCallback);
            m_PresetDropdown.name = "preset-dropdown";
            m_PresetDropdown.tooltip = "Select your game type";
            return m_PresetDropdown;
        }

        void RaiseValueChangedCallback(ChangeEvent<Enum> eventData)
        {
            if (!Equals(eventData.newValue, eventData.previousValue))
            {
                Undo.RecordObject(UserChoicesObject.instance, "Selected Preset");
                var newVal = (Preset) eventData.newValue;
                UserChoicesObject.instance.Preset = newVal;
                OnPresetSelected?.Invoke(newVal);
            }
        }
        
        public void SetAdvancedSectionVisible (bool isVisible){
            var advanced = ContentRoot.Q<Foldout>(k_AdvancedFoldoutName);
            advanced.style.display = isVisible ? new StyleEnum<DisplayStyle>(DisplayStyle.Flex) : new StyleEnum<DisplayStyle>(DisplayStyle.None);
        }
    }
}
