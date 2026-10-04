using System;
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using Unity.Multiplayer.Center.Common;
using Unity.Multiplayer.Center.Questionnaire;
using Unity.Multiplayer.Center.Recommendations;
using UnityEngine;
using Object = UnityEngine.Object;

namespace Unity.MultiplayerCenterTests
{
    /// <summary>
    /// This is an integration test based on known data. It ensures that the recommendation matches the expected results.
    /// </summary>
    [TestFixture]
    partial class RecommendationTests
    {
        [SetUp]
        [TearDown]
        public void Setup()
        {
            Object.DestroyImmediate(RecommenderSystemDataObject.instance); // force reload from disk if accessed
        }
        
        /// <summary>
        /// Ensures that the packages recommended for a given preset match the expected packages (4 players)
        /// Note that this does not handle hidden dependencies
        /// </summary>
        [TestCase(Preset.None)]
        [TestCase(Preset.Adventure,
            "com.unity.netcode.gameobjects",
            "com.unity.multiplayer.playmode",
            "com.unity.multiplayer.widgets",
            "com.unity.multiplayer.tools",
            "com.unity.dedicated-server",
            "com.unity.services.multiplayer",
            "com.unity.services.vivox")]
        [TestCase(Preset.Shooter,
            "com.unity.netcode",
            "com.unity.entities.graphics",
            "com.unity.multiplayer.widgets",
            "com.unity.multiplayer.playmode",
            "com.unity.services.multiplayer",
            "com.unity.services.vivox")]
        [TestCase(Preset.Racing,
            "com.unity.netcode",
            "com.unity.entities.graphics",
            "com.unity.multiplayer.widgets",
            "com.unity.multiplayer.playmode", 
            "com.unity.services.multiplayer",
            "com.unity.services.vivox")]
        [TestCase(Preset.TurnBased,
            "com.unity.services.cloudcode",
            "com.unity.services.deployment",
            "com.unity.multiplayer.playmode",
            "com.unity.services.multiplayer",
            "com.unity.services.vivox")]
        [TestCase(Preset.Simulation,
            "com.unity.netcode.gameobjects",
            "com.unity.multiplayer.playmode",
            "com.unity.multiplayer.widgets",
            "com.unity.multiplayer.tools",
            "com.unity.services.multiplayer",
            "com.unity.services.vivox")]
        [TestCase(Preset.Strategy,
            "com.unity.multiplayer.playmode",
            "com.unity.services.multiplayer",
            "com.unity.transport",
            "com.unity.services.vivox")]
        [TestCase(Preset.Sports,
            "com.unity.netcode",
            "com.unity.entities.graphics",
            "com.unity.multiplayer.widgets",
            "com.unity.multiplayer.playmode",
            "com.unity.services.multiplayer",
            "com.unity.services.vivox")]
        [TestCase(Preset.RolePlaying,
            "com.unity.multiplayer.playmode",
            "com.unity.services.multiplayer",
            "com.unity.transport",
            "com.unity.services.vivox")]
        [TestCase(Preset.Async,
            "com.unity.services.cloudcode",
            "com.unity.services.deployment",
            "com.unity.services.vivox",
            "com.unity.services.multiplayer",
            "com.unity.multiplayer.playmode")]
        [TestCase(Preset.Fighting,
            "com.unity.multiplayer.playmode",
            "com.unity.services.multiplayer",
            "com.unity.transport",
            "com.unity.services.vivox")]
        [TestCase(Preset.Sandbox,
            "com.unity.netcode.gameobjects",
            "com.unity.multiplayer.playmode",
            "com.unity.multiplayer.widgets",
            "com.unity.multiplayer.tools",
            "com.unity.services.multiplayer",
            "com.unity.services.vivox")]
        public void TestPreset_RecommendedPackagesMatchesExpected(Preset preset, params string[] expected)
        {
            var recommendation = UtilsForRecommendationTests.ComputeRecommendationForPreset(preset);
            var allPackages = RecommenderSystem.GetSolutionsToRecommendedPackageViewData();
            if (expected == null || expected.Length == 0)
            {
                Assert.IsNull(recommendation);
                return;
            }

            var actualRecommendedPackages = RecommendationUtils.PackagesToInstall(recommendation, allPackages)
                .Select(e => e.PackageId).ToArray();

            // Use AreEqual instead of AreEquivalent to get a better error message
            Array.Sort(expected);
            Array.Sort(actualRecommendedPackages);
            CollectionAssert.AreEqual(expected, actualRecommendedPackages);
        }

        [TestCaseSource(nameof(AdventurePresetCases))]
        [TestCaseSource(nameof(SandboxPresetCases))]
        [TestCaseSource(nameof(AsyncPresetCases))]
        [TestCaseSource(nameof(TurnBasedPresetCases))]
        [TestCaseSource(nameof(FightingPresetCases))]
        [TestCaseSource(nameof(RacingPresetCases))]
        [TestCaseSource(nameof(RolePlayingPresetCases))]
        [TestCaseSource(nameof(ShooterPresetCases))]
        [TestCaseSource(nameof(SimulationPresetCases))]
        [TestCaseSource(nameof(StrategyPresetCases))]
        [TestCaseSource(nameof(SportPresetCases))]
        public void TestPreset_RecommendedSolutionsAreValid(string playerCount, PossibleSolution netcode, PossibleSolution hosting, Preset preset)
        {
            var recommendation = UtilsForRecommendationTests.ComputeRecommendationForPreset(preset, playerCount: playerCount);
            Assert.NotNull(recommendation); 
            AssertTheRightSolutionsAreRecommended(netcode, hosting, recommendation);
            AssertAllDynamicReasonsAreProperlyFormed(recommendation);
        }

        // First line of table for case "cheating prevention not so important"
        [TestCase(PossibleSolution.DA, PossibleSolution.NGO, "NoCost", "Slow", "2", "4", "8" )]
        [TestCase(PossibleSolution.DA, PossibleSolution.NGO,"NoCost", "Fast", "2", "4", "8" )]
        [TestCase(PossibleSolution.DA, PossibleSolution.NGO,"BestMargin", "Slow", "2", "4", "8" )]   
        [TestCase(PossibleSolution.DA, PossibleSolution.NGO,"BestMargin", "Fast", "2", "4", "8" )]
        [TestCase(PossibleSolution.DA, PossibleSolution.NGO,"BestExperience", "Slow", "2", "4", "8")]
        [TestCase(PossibleSolution.DS, PossibleSolution.N4E,"BestExperience", "Fast", "2","4", "8")]
        
        // Second line
        [TestCase(PossibleSolution.DA, PossibleSolution.NGO,"NoCost", "Slow", "16", "64+" )]
        [TestCase(PossibleSolution.DA, PossibleSolution.N4E,"NoCost", "Fast", "16", "64+" )]
        [TestCase(PossibleSolution.DA, PossibleSolution.NGO,"BestMargin", "Slow", "16", "64+" )]
        [TestCase(PossibleSolution.DS, PossibleSolution.N4E,"BestMargin", "Fast", "16", "64+" )]
        [TestCase(PossibleSolution.DS, PossibleSolution.N4E,"BestExperience", "Slow", "16", "64+")]
        [TestCase(PossibleSolution.DS, PossibleSolution.N4E,"BestExperience", "Fast", "16", "64+")]
        
        // Third line
        [TestCase(PossibleSolution.DS, PossibleSolution.N4E,"NoCost", "Slow", "128", "256", "512" )]
        [TestCase(PossibleSolution.DS, PossibleSolution.N4E,"NoCost", "Fast", "128", "256", "512" )]
        [TestCase(PossibleSolution.DS, PossibleSolution.N4E,"BestMargin", "Slow", "128", "256", "512" )]
        [TestCase(PossibleSolution.DS, PossibleSolution.N4E,"BestMargin", "Fast", "128", "256", "512" )]
        [TestCase(PossibleSolution.DS, PossibleSolution.N4E,"BestExperience", "Slow", "128", "256", "512")]
        [TestCase(PossibleSolution.DS, PossibleSolution.N4E,"BestExperience", "Fast", "128", "256", "512")]
        public void TestGameSpecsForClientServerWithoutPreset_CheatingNotImportant_MatchesMiroTable(PossibleSolution expectedHosting,
            PossibleSolution expectedNetcode, string costSensitivity, string pace, params string[] playerCounts)
        {
            foreach (var playerCount in playerCounts)
            {
                var questionnaireData = UtilsForRecommendationTests.GetProjectQuestionnaire();
                var answerData = GenerateAnswerDataForClientServer(false, pace, playerCount, costSensitivity);
                var recommendation = RecommenderSystem.GetRecommendation(questionnaireData, answerData);
                Assert.NotNull(recommendation);
                var msg = $"{pace}, {costSensitivity}, {playerCount} players: ";
                AssertTheRightSolutionsAreRecommended(expectedNetcode, expectedHosting, recommendation, msg);
            }
        }
        
        // First line of table for case "cheating prevention important"
        [TestCase( PossibleSolution.NGO, "NoCost", "Slow", "2", "4", "8" )]
        [TestCase(PossibleSolution.N4E,"NoCost", "Fast", "2", "4", "8" )]
        [TestCase(PossibleSolution.NGO,"BestMargin", "Slow", "2", "4", "8" )]   
        [TestCase(PossibleSolution.N4E,"BestMargin", "Fast", "2", "4", "8" )]
        [TestCase(PossibleSolution.NGO,"BestExperience", "Slow", "2", "4", "8")]
        [TestCase(PossibleSolution.N4E,"BestExperience", "Fast", "2", "4", "8")]
        
        // Second line
        [TestCase(PossibleSolution.N4E,"NoCost", "Slow", "16", "64+" )]
        [TestCase(PossibleSolution.N4E,"NoCost", "Fast", "16", "64+" )]
        [TestCase(PossibleSolution.NGO,"BestMargin", "Slow", "16", "64+" )]
        [TestCase(PossibleSolution.N4E,"BestMargin", "Fast", "16", "64+" )]
        [TestCase(PossibleSolution.N4E,"BestExperience", "Slow", "16", "64+")]
        [TestCase(PossibleSolution.N4E,"BestExperience", "Fast", "16", "64+")]
        
        // Third line
        [TestCase(PossibleSolution.N4E,"NoCost", "Slow", "128", "256", "512" )]
        [TestCase(PossibleSolution.N4E,"NoCost", "Fast", "128", "256", "512" )]
        [TestCase(PossibleSolution.N4E,"BestMargin", "Slow", "128", "256", "512" )]
        [TestCase(PossibleSolution.N4E,"BestMargin", "Fast", "128", "256", "512" )]
        [TestCase(PossibleSolution.N4E,"BestExperience", "Slow", "128", "256", "512")]
        [TestCase(PossibleSolution.N4E,"BestExperience", "Fast", "128", "256", "512")]
        public void TestGameSpecsForClientServerWithoutPreset_CheatingImportant_MatchesMiroTable(
            PossibleSolution expectedNetcode, string costSensitivity, string pace, params string[] playerCounts)
        {
            const PossibleSolution expectedHosting = PossibleSolution.DS; // for all cases
            foreach (var playerCount in playerCounts)
            {
                var questionnaireData = UtilsForRecommendationTests.GetProjectQuestionnaire();
                var answerData = GenerateAnswerDataForClientServer(true, pace, playerCount, costSensitivity);
                var recommendation = RecommenderSystem.GetRecommendation(questionnaireData, answerData);
                Assert.NotNull(recommendation);
                var msg = $"{pace}, {costSensitivity}, {playerCount} players: ";
                AssertTheRightSolutionsAreRecommended(expectedNetcode, expectedHosting, recommendation, msg);
            }
        }

        static AnswerData GenerateAnswerDataForClientServer(bool cheatingImportant, string pace, string playerCount, string costSensitivity)
        {
            return new AnswerData(){ Answers = new List<AnsweredQuestion>
            {
                new () { QuestionId = "CostSensitivity", Answers = new() {costSensitivity}},
                new () { QuestionId = "Pace", Answers = new() {pace}},
                new () { QuestionId = "PlayerCount", Answers = new() {playerCount}},
                new () { QuestionId = "NetcodeArchitecture", Answers = new() {"ClientServer"}},
                new () { QuestionId = "Cheating", Answers = new() {cheatingImportant ? "CheatingImportant" : "CheatingNotImportant"}}
            }};
        }   
        
        [Test]
        public void PackageLists_PackagesHaveNames()
        {
            var allpackages = RecommenderSystemDataObject.instance.RecommenderSystemData.PackageDetailsById;
            foreach (var (id, details) in allpackages)
            {
                Assert.False(string.IsNullOrEmpty(details.Name), $"Package {id} has no name");
            }
        }

        [Test]
        public void PackageLists_DependenciesAreAllValid()
        {
            var allpackages = RecommenderSystemDataObject.instance.RecommenderSystemData.PackageDetailsById;
            foreach (var (id, details) in allpackages)
            {
                Assert.NotNull(details, $"Package {id} has no details in RecommenderSystemData.PackageDetails");
                if(details.AdditionalPackages == null)
                    continue;
                
                foreach (var additionalPackageId in details.AdditionalPackages)
                {
                    var additionalPackage = RecommendationUtils.GetPackageDetailForPackageId(additionalPackageId);
                    Assert.NotNull(additionalPackage, $"Package {id} has an invalid dependency: {additionalPackageId}. It should be added to RecommenderSystemData.PackageDetails");
                }
            }
        }

        [TestCase(PossibleSolution.NGO, PossibleSolution.LS, true)]
        [TestCase(PossibleSolution.NGO, PossibleSolution.DA, true)]
        [TestCase(PossibleSolution.NGO, PossibleSolution.DS, true)]
        [TestCase(PossibleSolution.NGO, PossibleSolution.CloudCode, true)] // ?
        
        [TestCase(PossibleSolution.N4E, PossibleSolution.LS, true)]
        [TestCase(PossibleSolution.N4E, PossibleSolution.DA, false)]
        [TestCase(PossibleSolution.N4E, PossibleSolution.DS, true)]
        [TestCase(PossibleSolution.N4E, PossibleSolution.CloudCode, true)] // ?
        
        [TestCase(PossibleSolution.CustomNetcode, PossibleSolution.LS, true)]
        [TestCase(PossibleSolution.CustomNetcode, PossibleSolution.DA, false)]
        [TestCase(PossibleSolution.CustomNetcode, PossibleSolution.DS, true)]
        [TestCase(PossibleSolution.CustomNetcode, PossibleSolution.CloudCode, true)] // ?
        
        [TestCase(PossibleSolution.NoNetcode, PossibleSolution.LS, true)]
        [TestCase(PossibleSolution.NoNetcode, PossibleSolution.DA, false)] // ?
        [TestCase(PossibleSolution.NoNetcode, PossibleSolution.DS, true)]
        [TestCase(PossibleSolution.NoNetcode, PossibleSolution.CloudCode, true)]
        public void TestIncompatibilityWithSolution_MatchesExpected(PossibleSolution netcode, PossibleSolution hostingModel, bool expected)
        {
            var recommendationData = RecommenderSystemDataObject.instance.RecommenderSystemData;
            var actual = recommendationData.IsHostingModelCompatibleWithNetcode(netcode, hostingModel, out string reason);
            Assert.AreEqual(expected, actual, $"Hosting model {hostingModel} should be {(expected ? "compatible" : "incompatible")} with netcode {netcode}");
            Assert.AreEqual(expected, string.IsNullOrEmpty(reason), "reason is " + reason);
        }
        
        // Additional packages are not used in version 1.0.0. We suspect however that killing the feature is not wise. 
        // Therefore, we check that the logic is intact
        [Test]
        public void TestAdditionalPackagesStillWork()
        {
            var packageDetails = RecommenderSystemDataObject.instance.RecommenderSystemData.PackageDetailsById;
            
            // nonsensical change, but with existing package.
            packageDetails["com.unity.netcode"].AdditionalPackages = new[] {"com.unity.netcode.gameobjects"};
            
            var mainPackages = new List<RecommendedPackageViewData>
            {
                new (packageDetails["com.unity.netcode"], RecommendationType.MainArchitectureChoice, null),
                new (packageDetails["com.unity.services.multiplayer"], RecommendationType.HostingFeatured, null)
            };
            var expectedPackages = new List<string> {"com.unity.netcode", "com.unity.services.multiplayer", 
                "com.unity.netcode.gameobjects", // added as additional package
                "com.unity.multiplayer.center.quickstart"}; // always added

            RecommendationUtils.GetPackagesWithAdditionalPackages(mainPackages, out var allPackages, out var allNames, out var tooltip);
            expectedPackages.Sort();
            allPackages.Sort();
            CollectionAssert.AreEqual(expectedPackages, allPackages);
            
            CollectionAssert.Contains(allNames, "Netcode for GameObjects");
            Assert.True(tooltip.Contains("Netcode for GameObjects"));
            
            Assert.AreEqual(allPackages.Count -1, allNames.Count, "Quickstart should not be included in the names");
        }
        
        static void AssertTheRightSolutionsAreRecommended(PossibleSolution expectedNetcode, PossibleSolution expectedHosting, RecommendationViewData recommendation, string msg=null)
        {
            AssertHighestScoreSolution(expectedNetcode, recommendation.NetcodeOptions, msg);
            AssertRightSolution(expectedNetcode, recommendation.NetcodeOptions, msg);
            if(expectedNetcode != PossibleSolution.NGO && expectedHosting == PossibleSolution.DA)
                AssertHighestScoreSolution(expectedHosting, recommendation.ServerArchitectureOptions, msg);
            else 
                AssertRightSolution(expectedHosting, recommendation.ServerArchitectureOptions, msg);
        }

        static void AssertAllDynamicReasonsAreProperlyFormed(RecommendationViewData recommendation)
        {
            foreach (var hostingModelRecommendation in recommendation.ServerArchitectureOptions)
            {
                AssertHasProperDynamicReason(hostingModelRecommendation);
            }
            
            foreach (var netcodeRecommendation in recommendation.NetcodeOptions)
            {
                AssertHasProperDynamicReason(netcodeRecommendation);
            }
        }

        static void AssertHasProperDynamicReason(RecommendedSolutionViewData solution)
        {
            Assert.False(solution.Reason.Contains(Scoring.DynamicKeyword), $"Reason for {solution.Solution} contain dynamic keyword");
            Assert.True(solution.Reason.Length > 10, $"Reason for {solution.Solution} is too short ({solution.Reason})");
            Assert.False(solution.Reason.EndsWith(".."), $"Reason for {solution.Solution} ends with two dots ({solution.Reason})");
            Assert.True(solution.Reason.EndsWith("."), $"Reason for {solution.Solution} does not end with a dot ({solution.Reason})");
        }
        
        static void AssertRightSolution(PossibleSolution expectedNetcode, RecommendedSolutionViewData[] data, string msg="")
        {
            var selectedView = data.FirstOrDefault(e => e.Selected);
            Assert.NotNull(selectedView, $"{msg}No solution selected");
            Assert.AreEqual(expectedNetcode, selectedView.Solution,
                $"{msg}Expected {expectedNetcode} but got {selectedView.Solution} selected.{string.Join(", ", data.Select(e => $"{e.Solution}: {e.Score}"))}");
        }

        static void AssertHighestScoreSolution(PossibleSolution expectedNetcode, RecommendedSolutionViewData[] data, string msg = "")
        {
            var maxScore = data.Max(e => e.Score);
            var solutionsWithMax = data.Where(e => Mathf.Approximately(e.Score, maxScore));
            Assert.AreEqual(1, solutionsWithMax.Count(), $"{msg}Multiple solutions with max score");
            var solutionWithMax = solutionsWithMax.First();
            Assert.True(solutionWithMax.RecommendationType is RecommendationType.MainArchitectureChoice or RecommendationType.Incompatible, $"The solution with the max score does not have the right recommendation type ({solutionWithMax.RecommendationType}");
            Assert.AreEqual(expectedNetcode, solutionWithMax.Solution, $"{msg}Expected {expectedNetcode} but highest score was {solutionWithMax.Solution}.{string.Join(", ", data.Select(e => $"{e.Solution}: {e.Score}"))}");
        }
    }
}
