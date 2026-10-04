using System;
using Unity.Multiplayer.Center.Common;
using UnityEditor;
using UnityEngine;

namespace Unity.Multiplayer.Center.Questionnaire
{
    /// <summary>
    /// The unity object that contains the current choices of the user.
    /// </summary>
    [FilePath("Assets/UserChoices.choices", FilePathAttribute.Location.ProjectFolder)]
    internal class UserChoicesObject : ScriptableSingleton<UserChoicesObject>
    {
        /// <summary>
        /// The version of the questionnaire the answers correspond to.
        /// </summary>
        public string QuestionnaireVersion;
        
        /// <summary>
        /// The answers of the user in the Game specs questionnaire.
        /// </summary>
        public AnswerData UserAnswers = new();

        /// <summary>
        /// Current preset selected by the user.
        /// </summary>
        public Preset Preset;
        
        /// <summary>
        /// The main selections made by the user in the recommendation tab.
        /// </summary>
        public SelectedSolutionsData SelectedSolutions;

        /// <summary>
        /// Raised when the SelectedSolutions changes
        /// </summary>
        public event Action OnSolutionSelectionChanged;
        
        /// <summary>
        /// Set the user selection and calls OnSelectionChanged if needed
        /// </summary>
        /// <param name="hostingModel">The selected hosting model</param>
        /// <param name="netcodeSolution">The selected netcode solution</param>
        internal void SetUserSelection(SelectedSolutionsData.HostingModel hostingModel, SelectedSolutionsData.NetcodeSolution netcodeSolution)
        {
            SelectedSolutions.SelectedHostingModel = hostingModel;
            SelectedSolutions.SelectedNetcodeSolution = netcodeSolution;
            OnSolutionSelectionChanged?.Invoke();
        }
        
        /// <summary>
        /// Save to disk (see filepath)
        /// </summary>
        internal void Save()
        {
            QuestionnaireVersion = QuestionnaireObject.instance.Questionnaire.Version;
            this.Save(true); 
        }
        
        internal string FilePath => GetFilePath();

    }
}
