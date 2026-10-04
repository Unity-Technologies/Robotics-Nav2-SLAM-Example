using System;
using Unity.Multiplayer.Center.Common.Analytics;
using UnityEngine.Analytics;

namespace Unity.Multiplayer.Center.Analytics
{
    /// <summary>
    /// Package representation in the analytics data.
    /// </summary>
    [Serializable]
    internal struct Package
    {
        /// <summary>
        /// The identifier of the package.
        /// </summary>
        public string PackageId;

        /// <summary>
        /// Whether the user has selected this package for installation.
        /// </summary>
        public bool SelectedForInstall;

        /// <summary>
        /// Whether the package was recommended.
        /// </summary>
        public bool IsRecommended;

        /// <summary>
        /// Whether the package was already installed when the installation attempt event occured
        /// </summary>
        public bool IsAlreadyInstalled;
    }

    /// <summary>
    /// A single Answer to the GameSpecs questionnaire.
    /// </summary>
    [Serializable]
    internal struct GameSpec
    {
        /// <summary>
        /// The identifier of the answered question (does not change).
        /// </summary>
        public string QuestionId;

        /// <summary>
        /// The text of the question as displayed in the UI (may change with versions).
        /// </summary>
        public string QuestionText;

        /// <summary>
        /// Whether the question accepts multiple answers.
        /// </summary>
        public bool AcceptsMultipleAnswers;

        /// <summary>
        /// The identifier of the answered question (does not change).
        /// </summary>
        public string AnswerId;

        /// <summary>
        /// The text of the answer as displayed in the UI (may change with versions).
        /// </summary>
        public string AnswerText;
    }

    /// <summary>
    ///
    /// </summary>
    [Serializable]
    internal struct RecommendationData : IAnalytic.IData
    {
        /// <summary>
        /// The preset selected by the user.
        /// </summary>
        public int Preset;

        /// <summary>
        /// The preset selected by the user (game genre) as displayed in the UI.
        /// </summary>
        public string PresetName;

        /// <summary>
        /// The version defined in the Questionnaire data.
        /// </summary>
        public string QuestionnaireVersion;

        /// <summary>
        /// All the selected answers to the questions of the game specs questionnaire.
        /// </summary>
        public GameSpec[] GameSpecs;
    }

    /// <summary>
    /// What type of content the user Interacted with (buttons).
    /// </summary>
    [Serializable]
    internal struct InteractionData : IAnalytic.IData
    {
        /// <summary>
        /// The identifier of the section that contains the button.
        /// </summary>
        public string SectionId;
        
        /// <summary>
        /// Whether it is a call to action or a link.
        /// </summary>
        public InteractionDataType Type;
        
        /// <summary>
        /// The name of the button in the UI.
        /// </summary>
        public string DisplayName;
        
        /// <summary>
        /// The target package for which the section is helpful.
        /// </summary>
        public string TargetPackageId;
    }

    /// <summary>
    /// Payload of the installation event.
    /// </summary>
    [Serializable]
    internal struct InstallData : IAnalytic.IData
    {
        /// <summary>
        /// The preset selected by the user.
        /// </summary>
        public int Preset;
        
        /// <summary>
        /// The preset selected by the user (game genre) as displayed in the UI.
        /// </summary>
        public string PresetName;
        
        /// <summary>
        /// The version defined in the Questionnaire data.
        /// </summary>
        public string QuestionnaireVersion;
        
        /// <summary>
        /// All the selected answers to the questions of the game specs questionnaire.
        /// </summary>
        public GameSpec[] GamesSpecs;
        
        /// <summary>
        /// The packages that were in the recommendation tab of the multiplayer center
        /// </summary>
        public Package[] Packages;

        /// <summary>
        /// The hosting model selected by the user as displayed in the UI.
        /// </summary>
        public string hostingModelName;

        /// <summary>
        /// The hosting model is the recommended solution.
        /// </summary>
        public bool hostingModelRecommended;

        /// <summary>
        /// The netcode solution selected by the user as displayed in the UI.
        /// </summary>
        public string netcodeSolutionName;

        /// <summary>
        /// The netcode solution is the recommended solution.
        /// </summary>
        public bool netcodeSolutionRecommended;
    }
}
