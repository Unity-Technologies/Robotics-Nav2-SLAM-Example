using System;
using System.Collections.Generic;
using Unity.Multiplayer.Center.Common;
using Unity.Multiplayer.Center.Questionnaire;
using UnityEditor;
using UnityEngine;
using UnityEngine.UIElements;
using Toggle = UnityEngine.UIElements.Toggle;

namespace Unity.Multiplayer.Center.Window.UI
{
    /// <summary>
    /// Creates visual elements that represent a question / the overview of the questionnaire
    /// </summary>
    internal static class QuestionViewFactory
    {
        public static VisualElement CreateOverview(QuestionnaireData questionnaire, int currentQuestionIndex)
        {
            if (currentQuestionIndex < 0 || currentQuestionIndex >= questionnaire.Questions.Length)
                throw new ArgumentOutOfRangeException(nameof(currentQuestionIndex));

            var label = new Label($"Question {currentQuestionIndex + 1} of {questionnaire.Questions.Length}");
            label.AddToClassList("questionnaireOverview");
            return label;
        }

        public static VisualElement StandardQuestionHeader(Question question, out VisualElement inputContainer)
        {
            var root = new VisualElement();
            root.AddToClassList(StyleClasses.QuestionView);
            if (question.IsMandatory)
            {
                root.AddToClassList(StyleClasses.MandatoryQuestion);
            }

            var title = new Label(question.Title);
            title.tooltip = question.Description;
            root.Add(title);
            inputContainer = root;
            return root;
        }

        public static VisualElement CreateQuestionViewInput(Question question, AnsweredQuestion answer, Action<AnsweredQuestion> onAnswerChanged)
        {
            return question.ViewType switch
            {
                ViewType.Toggle => CreateToggle(question, answer, onAnswerChanged),
                ViewType.Radio => CreateRadio(question, answer, onAnswerChanged),
                ViewType.Checkboxes => CreateCheckboxes(question, answer, onAnswerChanged),
                ViewType.DropDown => CreateDropDown(question, answer, onAnswerChanged),
                _ => throw new ArgumentOutOfRangeException()
            };
        }

        static VisualElement CreateDropDown(Question question, AnsweredQuestion answer, Action<AnsweredQuestion> onAnswerChanged)
        {
            var dropdown = new DropdownField();
            var choices = new List<string>(question.Choices.Length);
            foreach (var choice in question.Choices)
            {
                choices.Add(choice.Title);
            }
            dropdown.choices = choices;
            
            if (answer.Answers?.Count > 0)
            {
                var currentAnswer = Array.Find(question.Choices, choice => choice.Id == answer.Answers[0]);
                dropdown.SetValueWithoutNotify(currentAnswer.Title);
            }
            
            dropdown.RegisterValueChangedCallback(evt =>
            {
                RecordUndo();
                string selectedId = default;
                foreach (var choice in question.Choices)
                {
                    if (choice.Title == evt.newValue)
                    {
                        selectedId = choice.Id;
                        break;
                    }
                }

                answer.Answers = new List<string>(1) {selectedId};
                onAnswerChanged?.Invoke(answer);
            });            
            return dropdown;
            
        }
        
        public static VisualElement CreateCheckboxes(Question question, AnsweredQuestion answeredQuestion, Action<AnsweredQuestion> onAnswerChanged = null)
        {
            var root = new VisualElement();

            foreach (var possibleAnswer in question.Choices)
            {
                var toggle = new Toggle(possibleAnswer.Title);
                toggle.tooltip = possibleAnswer.Description;
                var answerCopy = possibleAnswer;
                toggle.RegisterValueChangedCallback(evt => UpdateAnswersWithCheckBoxes(answeredQuestion, answerCopy.Id, evt.newValue, onAnswerChanged));
                root.Add(toggle);
            }

            return root;
        }

        static void UpdateAnswersWithCheckBoxes(AnsweredQuestion question, string id, bool newValue, Action<AnsweredQuestion> onAnswerChanged)
        {
            RecordUndo();
            if (newValue && !question.Answers.Contains(id))
            {
                question.Answers.Add(id);
                onAnswerChanged?.Invoke(question);
            }
            else if (!newValue && question.Answers.Contains(id))
            {
                question.Answers.Remove(id);
                onAnswerChanged?.Invoke(question);
            }
        }

        public static RadioButtonGroup CreateRadio(Question question, AnsweredQuestion answeredQuestion, Action<AnsweredQuestion> onAnswerChanged = null)
        {
            var group = new RadioButtonGroup();

            foreach (var q in question.Choices)
            {
                var radioButton = new RadioButton(q.Title);
                radioButton.tooltip = q.Description;

                // Todo: just checking for the first question for now.
                if (answeredQuestion.Answers != null && q.Id == answeredQuestion.Answers[0])
                    radioButton.SetValueWithoutNotify(true);
                group.Add(radioButton);
            }

            // this was not working, so I had to use the foreach loop above

            // var answerIndex = question.Choices.ToList().FindIndex(c => c.Id == answeredQuestion.Answers[0]);
            // group.SetValueWithoutNotify(answerIndex);

            group.RegisterValueChangedCallback(evt =>
            {
                RecordUndo();
                answeredQuestion.Answers = new List<string>(1) {question.Choices[evt.newValue].Id};
                onAnswerChanged?.Invoke(answeredQuestion);
            });

            return group;
        }

        public static VisualElement CreateToggle(Question question, AnsweredQuestion answeredQuestion, Action<AnsweredQuestion> onAnswerChanged)
        {
            return CreateRadio(question, answeredQuestion, onAnswerChanged);
        }

        static void RecordUndo()
        {
            Undo.RecordObject(UserChoicesObject.instance,"Game Spec Change");
        }
    }
}
