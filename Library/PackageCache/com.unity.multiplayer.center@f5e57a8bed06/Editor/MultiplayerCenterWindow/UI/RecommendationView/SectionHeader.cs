using System;
using System.Collections.Generic;
using Unity.Multiplayer.Center.Recommendations;
using UnityEngine;
using UnityEngine.UIElements;

namespace Unity.Multiplayer.Center.Window.UI.RecommendationView
{
    internal class SectionHeader : VisualElement
    {
        readonly DropdownField m_MainDropdown;
        readonly Label m_MainPackageDescription;
        readonly VisualElement m_PosterImageIcon;
        public event Action OnSolutionSelected;
        const string k_ItemIsRecommendedAppend = " - Recommended";

        RecommendedSolutionViewData[] m_Solutions;

        public SectionHeader(string headlineLabel)
        {
            var posterImageContainer = new VisualElement(){name = "card-poster-image"};
            m_PosterImageIcon = new VisualElement();
            posterImageContainer.Add(m_PosterImageIcon);
            
            m_MainDropdown = new DropdownField();
            m_MainPackageDescription = new Label();
            var headline = new Label(){text = headlineLabel, name = "card-headline"};
            Add(posterImageContainer);
            Add(headline);
            Add(m_MainDropdown);
            Add(m_MainPackageDescription);
            m_MainDropdown.RegisterValueChangedCallback(OnItemSelected);
        }

        public void UpdateData(RecommendedSolutionViewData[] availableSolutions)
        {
            m_Solutions = availableSolutions;
            m_MainDropdown.choices = GenerateChoices(availableSolutions, out var selectedSolution, out var solutionTitleWithAppend);
            var iconClass = "icon-" + selectedSolution.Solution;
            m_PosterImageIcon.ClearClassList();
            m_PosterImageIcon.AddToClassList(iconClass);
            m_MainDropdown.SetValueWithoutNotify(solutionTitleWithAppend);
            m_MainPackageDescription.text = selectedSolution.Reason;
        }

        /// <summary>
        /// Returns a list of choices for the dropdown and appends k_ItemIsRecommendedAppend to the recommended choice.
        /// </summary>
        /// <param name="availableSolutions">All available Solutions</param>
        /// <param name="selectedSolution"> The solution that is selected.</param>
        /// <param name="selectedSolutionTitleAppended">The title of the selection solution with appended k_ItemIsRecommendedAppend</param>
        /// <returns>List of Choices to be used with Dropdown</returns>
        static List<string> GenerateChoices(RecommendedSolutionViewData[] availableSolutions, out RecommendedSolutionViewData selectedSolution, out string selectedSolutionTitleAppended)
        {
            var choices = new List<string>(availableSolutions.Length);
            selectedSolutionTitleAppended = null;
            selectedSolution = null;

            foreach (var sol in availableSolutions)
            {
                if(sol.RecommendationType == RecommendationType.Incompatible)
                    continue;

                var isRecommended = sol.RecommendationType == RecommendationType.MainArchitectureChoice;
                choices.Add(isRecommended ? sol.Title + k_ItemIsRecommendedAppend : sol.Title);

                if (sol.Selected)
                {
                    selectedSolutionTitleAppended = choices[^1];
                    selectedSolution = sol;
                }
            }

            return choices;
        }

        static string RemoveRecommendationString(string choice)
        {
            return choice.Replace(k_ItemIsRecommendedAppend, "");
        }

        void OnItemSelected(ChangeEvent<string> evt)
        {
            foreach (var solution in m_Solutions)
            {
                solution.Selected = RemoveRecommendationString(evt.newValue) == solution.Title;
            }
            OnSolutionSelected?.Invoke();
        }
    }
}
