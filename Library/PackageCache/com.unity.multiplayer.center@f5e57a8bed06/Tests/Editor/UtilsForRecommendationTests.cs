using System;
using System.Linq;
using NUnit.Framework;
using Unity.Multiplayer.Center.Common;
using Unity.Multiplayer.Center.Questionnaire;
using Unity.Multiplayer.Center.Recommendations;
using UnityEngine;

namespace Unity.MultiplayerCenterTests
{
    internal static class UtilsForRecommendationTests
    {
        public static QuestionnaireData GetProjectQuestionnaire()
        {
            // TODO: maybe accessing from disk is a better idea than using the singleton? (side effects)
            var result = Clone(QuestionnaireObject.instance.Questionnaire);
            // sanity checks
            Assert.IsNotNull(result);
            Assert.IsNotNull(result.Questions);
            Assert.Greater(result.Questions.Length, 0);
            Assert.False(result.Questions.Any(x => x.Choices.Length == 0));
            return result;
        }

        public static AnswerData BuildAnswerMatching(QuestionnaireData questionnaireData)
        {
            var answerData = new AnswerData();
            foreach (var question in questionnaireData.Questions)
            {
                var middleChoice = question.Choices[question.Choices.Length / 2 - 1];
                var answeredQuestion = new AnsweredQuestion()
                {
                    QuestionId = question.Id,
                    Answers = new(){middleChoice.Id}
                };
                Logic.Update(answerData, answeredQuestion);
            }

            return answerData;
        }
        
        public static RecommendationViewData GetSomeRecommendation()
        {
            var questionnaireData = GetProjectQuestionnaire();
            var answerData = BuildAnswerMatching(questionnaireData);
            return RecommenderSystem.GetRecommendation(questionnaireData, answerData);
        }

        public static void AssertRecommendedSolutionNotNull(RecommendedSolutionViewData solution, bool checkMainPackage = true)
        {
            Assert.NotNull(solution);
            Assert.False(string.IsNullOrEmpty(solution.Title));
            Assert.That(solution.RecommendationType is RecommendationType.MainArchitectureChoice 
                or RecommendationType.SecondArchitectureChoice or RecommendationType.NotRecommended or RecommendationType.Incompatible, $"Recommendation type: {solution.RecommendationType}");
            if (checkMainPackage)
                Assert.NotNull(solution.MainPackage);
        }

        public static void AssertAllRecommendedPackageNotNull(SolutionsToRecommendedPackageViewData allPackages)
        {
            foreach (var selection in allPackages.Selections)
            {
                var packages = allPackages.GetPackagesForSelection(selection);
                var selectionString = $"Netcode {selection.Netcode} - Hosting {selection.HostingModel}";
                Assert.NotNull(packages, selectionString);
                CollectionAssert.IsNotEmpty(packages, selectionString);
                foreach (var package in packages)
                {
                    Assert.False(string.IsNullOrEmpty(package.Name), selectionString);
                    Assert.NotNull(string.IsNullOrEmpty(package.PackageId), $"{selectionString} - {package.Name}");
                    Assert.That(package.RecommendationType != RecommendationType.MainArchitectureChoice &&
                        package.RecommendationType != RecommendationType.SecondArchitectureChoice, $"{selectionString} - {package.Name}");    
                }
            }
        }

        /// <summary>
        /// All solutions should mention all packages, however the actual recommendations (RecommendationType) should be different.
        /// This checks that all packages in solution 1 are also in solution 2, but that they don't all have the same recommendation.
        /// Note: this a current requirement, but this might evolve. Adapt if necessary.
        /// 2nd Note: With the introduction of the Multiplayer SDK this requirement changed for the server architecture options, but not for the netcode options.
        /// </summary>
        /// <param name="reco1">Solution 1</param>
        /// <param name="reco2">Solution 2</param>
        public static void AssertSamePackagesWithDifferentRecommendations(RecommendedPackageViewData[] reco1,  RecommendedPackageViewData[] reco2, string msg)
        {
            Assert.AreEqual(reco1.Length, reco2.Length);
            var allTheSame = true;
            foreach (var p in reco1)
            {
                var matching = reco2.FirstOrDefault(x => x.PackageId == p.PackageId);
                Assert.IsNotNull(matching);
                if(matching.RecommendationType != p.RecommendationType)
                    allTheSame = false;
            }
            Assert.False(allTheSame, $"The two solutions have exactly the same recommended packages!"); 
        }

        public static T Clone<T>(T obj)
        {
            return JsonUtility.FromJson<T>(JsonUtility.ToJson(obj));
        }

        public static RecommendationViewData ComputeRecommendationForPreset(Preset preset, string playerCount = "4")
        {
            var questionnaireData = UtilsForRecommendationTests.GetProjectQuestionnaire();
            var answerData = GetAnswerDataForPreset(questionnaireData, preset, playerCount);
            var recommendation = RecommenderSystem.GetRecommendation(questionnaireData, answerData);
            return recommendation;
        }

        public static AnswerData GetAnswerDataForPreset(QuestionnaireData questionnaireData, Preset preset, string playerCount)
        {
            if (preset == Preset.None)
                return new AnswerData();

            var indexOfPreset = Array.IndexOf(questionnaireData.PresetData.Presets, preset);
            var matchingAnswers = questionnaireData.PresetData.Answers[indexOfPreset].Clone();
            var playerCountAnswer = new AnsweredQuestion()
            {
                QuestionId = "PlayerCount",
                Answers = new() {playerCount}
            };
            Logic.Update(matchingAnswers, playerCountAnswer);
            return matchingAnswers;
        }

        public static void SimulateSelectionChange(PossibleSolution newSelectedSolution, RecommendedSolutionViewData[] solutionViewDatas)
        {
            foreach (var solution in solutionViewDatas)
            {
                solution.Selected = solution.Solution == newSelectedSolution;
                if(solution.Selected && solution.RecommendationType == RecommendationType.Incompatible)
                    Assert.Fail("Selected incompatible solution");
            }
        }
    }
}
