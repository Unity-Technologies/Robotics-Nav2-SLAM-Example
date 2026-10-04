using System;
using System.Collections.Generic;
using System.Reflection;
using Unity.Multiplayer.Center.Common;
using Unity.Multiplayer.Center.Questionnaire;
using Unity.Multiplayer.Center.Recommendations;
using Unity.Multiplayer.Center.Window.UI;
using UnityEngine;

namespace Unity.Multiplayer.Center.Analytics
{
    internal static class AnalyticsUtils
    {
        // hard-coded to avoid recomputing every time / resizing arrays
        public const int NumNetcodePackage = 2;
        public const int NumHostingPackages = 1;

        /// <summary>
        /// From the recommendation view data (which contains the packages that the user sees and the user's selection),
        /// create the list of packages that will be sent to the analytics backend.
        /// </summary>
        /// <param name="data">The recommendation view data as shown in the recommendation tab</param>
        /// <param name="solutionToPackageData">The packages views</param>
        /// <returns>The list of packages to be sent along with the installation event.</returns>
        public static Package[] GetPackagesWithAnalyticsFormat(RecommendationViewData data, SolutionsToRecommendedPackageViewData solutionToPackageData)
        {
            var selectedNetcode = RecommendationUtils.GetSelectedNetcode(data);
            var selectedHostingModel = RecommendationUtils.GetSelectedHostingModel(data);
            var packages = solutionToPackageData.GetPackagesForSelection(selectedNetcode.Solution, selectedHostingModel.Solution);
            var packageCount = NumNetcodePackage + NumHostingPackages + packages.Length;
            
            var result = new Package[packageCount];
            var resultIndex = 0;

            AddSolutionPackages(data.NetcodeOptions, result, ref resultIndex);
            AddSolutionPackages(data.ServerArchitectureOptions, result, ref resultIndex);
            AddRecommendedPackages(packages, result, ref resultIndex);

            Debug.Assert(resultIndex == packageCount, $"Expected {packageCount} packages, got {resultIndex}");
            
            return result;
        }
        
        /// <summary>
        /// Fetches all the inspector name attributes of the Preset enum and returns the displayNames
        /// Important! It assumes the enum values are 0, ... , N 
        /// </summary>
        /// <returns>The array of preset names. The index in the array is the integer value of the enum value</returns>
        public static string[] GetPresetFullNames()
        {
            var t = typeof(Preset);
            var values = Enum.GetValues(t);
            var array = new string[values.Length];
            foreach (var value in values)
            {
                var preset = (Preset) value;
                var index = (int)preset;
                var asString = value.ToString();
                var memInfo = t.GetMember(asString);
                var attribute = memInfo[0].GetCustomAttribute<InspectorNameAttribute>(false);

                if (attribute != null)
                {
                    array[index] = attribute.displayName;
                }
                else
                {
                    Debug.LogError($"Could not fetch the full name of the preset value {asString}");
                    array[index] = asString;
                }
            }

            return array;
        }

        /// <summary>
        /// Converts AnswerData to game specs, providing the knowledge of the display names.
        /// It assumes there is exactly one answer in the answer list at this point.
        /// </summary>
        /// <param name="data">The answer data of the user</param>
        /// <param name="answerIdToAnswerName">Mapping answer id to display name</param>
        /// <param name="questionIdToQuestionName">Mapping question id to display name</param>
        /// <returns>The list of game spec that will be consumed by the analytics backend</returns>
        public static GameSpec[] ToGameSpecs(AnswerData data,
            IReadOnlyDictionary<string, string> answerIdToAnswerName,
            IReadOnlyDictionary<string, string> questionIdToQuestionName)
        {
            var result = new GameSpec[data.Answers.Count];
            for (var i = 0; i < result.Length; ++i)
            {
                var answer = data.Answers[i];
                var answerId = answer.Answers[0]; // TODO: make sure that this always exists
                result[i] = new GameSpec()
                {
                    QuestionId = answer.QuestionId,
                    QuestionText = questionIdToQuestionName[answer.QuestionId],
                    AcceptsMultipleAnswers = false, // TODO: add test that verifies this assumption
                    AnswerId = answerId,
                    AnswerText = answerIdToAnswerName[answerId]
                };
            }

            return result;
        }
        
        /// <summary>
        /// Creates the mapping from question id to question display name
        /// </summary>
        /// <param name="questionnaireData">The questionnaire data</param>
        /// <returns>The mapping</returns>
        public static IReadOnlyDictionary<string, string> GetQuestionDisplayNames(QuestionnaireData questionnaireData)
        {
            var dictionary = new Dictionary<string, string>();
            foreach (var question in questionnaireData.Questions)
            {
                dictionary[question.Id] = question.Title;
            }

            return dictionary;
        }
        
        /// <summary>
        /// Creates the mapping from answer id to answer display name
        /// </summary>
        /// <param name="questionnaireData">The questionnaire data</param>
        /// <returns>The mapping</returns>
        public static IReadOnlyDictionary<string, string> GetAnswerDisplayNames(QuestionnaireData questionnaireData)
        {
            var dictionary = new Dictionary<string, string>();
            foreach (var question in questionnaireData.Questions)
            {
                foreach (var answer in question.Choices)
                {
                    dictionary[answer.Id] = answer.Title;
                }
            }

            return dictionary;
        }

        static void AddSolutionPackages(RecommendedSolutionViewData[] options, Package[] result, ref int resultIndex)
        {
            foreach (var t in options)
            {
                if(string.IsNullOrEmpty(t.MainPackage?.PackageId))
                    continue;
                
                result[resultIndex] = new Package()
                {
                    PackageId = t.MainPackage.PackageId,
                    SelectedForInstall = t.Selected && t.RecommendationType != RecommendationType.Incompatible,
                    IsRecommended = t.RecommendationType is RecommendationType.MainArchitectureChoice,
                    IsAlreadyInstalled = t.MainPackage.IsInstalledAsProjectDependency
                };
                ++resultIndex;
            }
        }

        static void AddRecommendedPackages(RecommendedPackageViewData[] packageViewDatas, Package[] result, ref int resultIndex)
        {
            foreach (var viewData in packageViewDatas)
            {
                result[resultIndex] = new Package()
                {
                    PackageId = viewData.PackageId,
                    // TODO: remove hidden?
                    SelectedForInstall = viewData.Selected && viewData.RecommendationType != RecommendationType.Incompatible,
                    IsRecommended = viewData.RecommendationType.IsRecommendedPackage(),
                    IsAlreadyInstalled = viewData.IsInstalledAsProjectDependency
                };
                ++resultIndex;
            }
        }
    }
}
