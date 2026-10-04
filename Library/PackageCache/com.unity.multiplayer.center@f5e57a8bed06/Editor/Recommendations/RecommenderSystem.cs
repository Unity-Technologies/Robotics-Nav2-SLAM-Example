using System;
using System.Collections.Generic;
using Unity.Multiplayer.Center.Common;
using Unity.Multiplayer.Center.Questionnaire;
using UnityEngine;

namespace Unity.Multiplayer.Center.Recommendations
{
    using AnswerWithQuestion = Tuple<Question, Answer>;

    /// <summary>
    /// Builds recommendation views based on Questionnaire data and Answer data.
    /// The recommendation is based on the scoring of the answers, which is controlled by the RecommenderSystemData.
    /// </summary>
    internal static class RecommenderSystem
    {
        /// <summary>
        /// Main entry point for the recommender system: computes the recommendation based on the questionnaire data and
        /// the answers.
        /// If no answer has been given or the questionnaire does not match the answers, this returns null.
        /// </summary>
        /// <param name="questionnaireData">The questionnaire that the user filled.</param>
        /// <param name="answerData">The answers the user gave.</param>
        /// <returns>The recommendation view data.</returns>
        public static RecommendationViewData GetRecommendation(QuestionnaireData questionnaireData, AnswerData answerData)
        {
            var answers = CollectAnswers(questionnaireData, answerData);
            
            // Note: valid now only because we do not have multiple answers per question
            if (answers.Count < questionnaireData.Questions.Length) return null;

            var data = RecommenderSystemDataObject.instance.RecommenderSystemData;
            var scoredSolutions = CalculateScore(data, answers);

            return CreateRecommendation(data, scoredSolutions);
        }

        /// <summary>
        /// Get the view data for all possible solution selections.
        /// </summary>
        /// <returns>The constructed set of views</returns>
        public static SolutionsToRecommendedPackageViewData GetSolutionsToRecommendedPackageViewData()
        {
            var data = RecommenderSystemDataObject.instance.RecommenderSystemData;
            var installedPackageDictionary = PackageManagement.InstalledPackageDictionary();
            var selections = new SolutionSelection[16];
            var packages = new RecommendedPackageViewData[16][];
            PossibleSolution[] netcodes = { PossibleSolution.NGO, PossibleSolution.N4E, PossibleSolution.CustomNetcode, PossibleSolution.NoNetcode };
            PossibleSolution[] hostings = { PossibleSolution.LS, PossibleSolution.DS, PossibleSolution.CloudCode, PossibleSolution.DA };

            var index = 0;
            foreach (var netcode in netcodes)
            {
                foreach (var hosting in hostings)
                {
                    var selection = new SolutionSelection(netcode, hosting);
                    selections[index] = selection;
                    packages[index] = BuildRecommendationForSelection(data, selection, installedPackageDictionary);

                    ++index;
                }
            }

            return new SolutionsToRecommendedPackageViewData(selections, packages);
        }

        public static void AdaptRecommendationToNetcodeSelection(RecommendationViewData recommendation)
        {
            RecommendationUtils.MarkIncompatibleHostingModels(recommendation);
            var maxIndex = RecommendationUtils.IndexOfMaximumScore(recommendation.ServerArchitectureOptions);
            recommendation.ServerArchitectureOptions[maxIndex].RecommendationType = RecommendationType.MainArchitectureChoice;
            if (RecommendationUtils.GetSelectedHostingModel(recommendation) == null)
                recommendation.ServerArchitectureOptions[maxIndex].Selected = true;
        }

        static List<AnswerWithQuestion> CollectAnswers(QuestionnaireData questionnaireData, AnswerData answerData)
        {
            if (questionnaireData?.Questions == null || questionnaireData.Questions.Length == 0)
                throw new ArgumentException("Questionnaire data is null or empty", nameof(questionnaireData));

            List<AnswerWithQuestion> givenAnswers = new();

            var answers = answerData.Answers;

            foreach (var answeredQuestion in answers)
            {
                // find question for the answer
                if (!Logic.TryGetQuestionByQuestionId(questionnaireData, answeredQuestion.QuestionId, out var question))
                    continue;

                // find answer object for the given answer id
                foreach (var answerId in answeredQuestion.Answers)
                {
                    if (!Logic.TryGetAnswerByAnswerId(question, answerId, out var choice))
                        continue;
                    givenAnswers.Add(Tuple.Create(question, choice));
                }
            }

            return givenAnswers;
        }

        static Dictionary<PossibleSolution, Scoring> CalculateScore(RecommenderSystemData data, List<AnswerWithQuestion> answers)
        {
            var possibleSolutions = Enum.GetValues(typeof(PossibleSolution));
            Dictionary<PossibleSolution, Scoring> scores = new(possibleSolutions.Length);

            foreach (var solution in possibleSolutions)
            {
                var solutionObject = data.SolutionsByType[(PossibleSolution) solution];
                scores.Add((PossibleSolution) solution, new Scoring(solutionObject.ShortDescription));
            }

            foreach (var (question, answer) in answers)
            {
                foreach (var scoreImpact in answer.ScoreImpacts)
                {
                    scores[scoreImpact.Solution].AddScore(scoreImpact.Score * question.GlobalWeight, scoreImpact.Comment);
                }
            }

            return scores;
        }

        static RecommendationViewData CreateRecommendation(RecommenderSystemData data, IReadOnlyDictionary<PossibleSolution, Scoring> scoredSolutions)
        {
            RecommendationViewData recommendation = new();
            var installedPackageDictionary = PackageManagement.InstalledPackageDictionary();

            recommendation.NetcodeOptions = BuildRecommendedSolutions(data, new [] {
                (PossibleSolution.NGO, scoredSolutions[PossibleSolution.NGO]), 
                (PossibleSolution.N4E, scoredSolutions[PossibleSolution.N4E]),
                (PossibleSolution.CustomNetcode, scoredSolutions[PossibleSolution.CustomNetcode]), 
                (PossibleSolution.NoNetcode, scoredSolutions[PossibleSolution.NoNetcode]) },
                installedPackageDictionary);

            recommendation.ServerArchitectureOptions = BuildRecommendedSolutions(data, new [] {
                (PossibleSolution.LS, scoredSolutions[PossibleSolution.LS]),
                (PossibleSolution.DS, scoredSolutions[PossibleSolution.DS]),
                (PossibleSolution.CloudCode, scoredSolutions[PossibleSolution.CloudCode]),
                (PossibleSolution.DA, scoredSolutions[PossibleSolution.DA]) },
                installedPackageDictionary);
            
            AdaptRecommendationToNetcodeSelection(recommendation);
            return recommendation;
        }

        static RecommendedSolutionViewData[] BuildRecommendedSolutions(RecommenderSystemData data, (PossibleSolution, Scoring)[] scoredSolutions, Dictionary<string, string> installedPackageDictionary)
        {
            var recommendedSolution = RecommendationUtils.FindRecommendedSolution(scoredSolutions);
            var result = new RecommendedSolutionViewData[scoredSolutions.Length];
            
            for (var index = 0; index < scoredSolutions.Length; index++)
            {
                var scoredSolution = scoredSolutions[index];
                var recoType = scoredSolution.Item1 == recommendedSolution ? RecommendationType.MainArchitectureChoice : RecommendationType.SecondArchitectureChoice;
                var reco = new RecommendedSolutionViewData(data, data.SolutionsByType[scoredSolution.Item1], recoType, scoredSolution.Item2, installedPackageDictionary);
                result[index] = reco;
            }

            return result;
        }

        static RecommendedPackageViewData[] BuildRecommendationForSelection(RecommenderSystemData data, SolutionSelection selection, Dictionary<string, string> installedPackageDictionary)
        {
            // Note: working on a copy that we modify
            var netcodePackages = (RecommendedPackage[]) data.SolutionsByType[selection.Netcode].RecommendedPackages.Clone();
            var hostingOverrides = data.SolutionsByType[selection.HostingModel].RecommendedPackages;
            foreach (var package in hostingOverrides)
            {
                var existing = Array.FindIndex(netcodePackages, p => p.PackageId == package.PackageId);
                if (existing == -1)
                {
                    Debug.LogError($"Malformed data for hosting model {selection.HostingModel}: package {package.PackageId} not found in netcode packages of {selection.Netcode}.");
                    continue;
                }

                netcodePackages[existing] = package;
            }
            
            var result = new RecommendedPackageViewData[netcodePackages.Length];
            for (var index = 0; index < netcodePackages.Length; index++)
            {
                var package = netcodePackages[index];
                installedPackageDictionary.TryGetValue(package.PackageId, out var installedVersion);
                result[index] = new RecommendedPackageViewData( data.PackageDetailsById[package.PackageId], package, installedVersion);
            }

            return result;
        }
    }
}
