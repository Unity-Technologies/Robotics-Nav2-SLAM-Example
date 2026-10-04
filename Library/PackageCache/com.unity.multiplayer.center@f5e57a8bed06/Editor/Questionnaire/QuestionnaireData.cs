using System;
using UnityEngine;

namespace Unity.Multiplayer.Center.Questionnaire
{
    /// <summary>
    /// The serializable data of the questionnaire
    /// </summary>
    [Serializable]
    internal class QuestionnaireData
    {
        /// <summary> The version of the format to serialize/deserialize data </summary>
        public string FormatVersion = "1.0.0";

        /// <summary> The version of the questionnaire itself (different questions, answer choice) </summary>
        public string Version ="1.2";

        /// <summary> All the questions in the right order (some might be hidden though) </summary>
        public Question[] Questions;
        
        /// <summary> The predefined answers for presets. The content should match the questions.</summary>
        public PresetData PresetData;
    }

    /// <summary>
    /// Possible multiplayer solution that needs to be scored in order to assess a match. Some are mutually exclusive,
    /// some are not.
    /// </summary>
    [Serializable]
    internal enum PossibleSolution
    {
        /// <summary> Netcode for GameObject, incompatible with N4E </summary>
        NGO,

        /// <summary> Netcode for Entities, incompatible with NGO </summary>
        N4E,

        /// <summary> Client Hosted Architecture (also called "Listen server"; using a host and not a dedicated server) </summary>
        LS,

        /// <summary> Dedicated server architecture (using a dedicated server and not a host) </summary>
        DS,
        
        /// <summary> Distributed authority (Authority will be distributed across different players)</summary> 
        DA,
        
        /// <summary> Not using Netcode for GameObjects nor Netcode for Entities </summary>
        CustomNetcode,
        
        /// <summary> Works asynchronously, with a database </summary> 
        NoNetcode,
        
        /// <summary> Recommended backend for async games, without a Netcode (goes with <see cref="NoNetcode"/>) </summary> 
        CloudCode
    }

    [Serializable]
    internal enum ViewType
    {
        /// <summary> Yes or No type of question best represented by a toggle</summary>
        Toggle,

        /// <summary> A question with multiple choices and where you can select only one answer</summary>
        Radio,

        /// <summary> A question with multiple choices and where you can select multiple answers</summary>
        Checkboxes,

        /// <summary> A question with a Drop Down</summary>
        DropDown
    }

    [Serializable]
    internal class Question
    {
        /// <summary> Id (unique across questions) </summary>
        public string Id;

        /// <summary> Short string to refer to the question (e.g. "Player Count") </summary>
        public string Title;

        /// <summary> Longer string to describe the question, which will be displayed in the tooltip </summary>
        public string Description;

        /// <summary> Optional weight to increase/decrease importance of this question, applied to all answers.</summary>
        //TODO: use ignore if default
        public float GlobalWeight = 1f;

        /// <summary> The type of view to use to display the question </summary>
        public ViewType ViewType;

        /// <summary> The possible answers to the question </summary>
        public Answer[] Choices;

        /// <summary> If the question is mandatory or not. Not overwritten by presets </summary>
        public bool IsMandatory;
    }

    [Serializable]
    internal class Answer
    {
        /// <summary> Id (unique across answers) </summary>
        public string Id;

        /// <summary> What is displayed to the user </summary>
        public string Title;
        
        /// <summary> Optional description that will be shown in a tooltip </summary>
        public string Description;

        /// <summary> How picking this answer will impact the score of a given solution </summary>
        public ScoreImpact[] ScoreImpacts;
    }

    [Serializable]
    internal class ScoreImpact
    {
        /// <summary> Which score is impacted </summary>
        public PossibleSolution Solution;

        /// <summary> Absolute value to add or subtract from the target score</summary>
        public float Score;

        /// <summary> A comment displayed to the user as for why this score is impacted </summary> 
        public string Comment;
    }
}