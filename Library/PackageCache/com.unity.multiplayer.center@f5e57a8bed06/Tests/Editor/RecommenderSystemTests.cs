using System;
using System.Linq;
using NUnit.Framework;
using Unity.Multiplayer.Center.Common;
using Unity.Multiplayer.Center.Questionnaire;
using Unity.Multiplayer.Center.Recommendations;

namespace Unity.MultiplayerCenterTests
{
    /// <summary>
    /// Basic test for the recommender system, without checking that the recommendation itself make sense.
    /// </summary>
    [TestFixture]
    class RecommenderSystemUnitTests
    {
        const string k_NoNetcodeTitle = "No Netcode";
        
        [SetUp]
        [TearDown]
        public void Cleanup()
        {
            UnityEngine.Object.DestroyImmediate(RecommenderSystemDataObject.instance); // force reload from disk if accessed
        }
        
        [Test]
        public void TestEmptyQuestionnaireAndAnswer_ThrowsArgumentException()
        {
            var questionnaireData = new QuestionnaireData();
            var answerData = new AnswerData();

            // Having an empty questionnaire is an invalid state and should be caught. 
            Assert.Throws<ArgumentException>(() => RecommenderSystem.GetRecommendation(questionnaireData, answerData));
        }

        [Test]
        public void TestEmptyQuestionnaire_ThrowsArgumentException()
        {
            var questionnaireData = new QuestionnaireData();
            var answerData = UtilsForRecommendationTests.BuildAnswerMatching(UtilsForRecommendationTests.GetProjectQuestionnaire());

            // Having an empty questionnaire is an invalid state and should be caught. 
            Assert.Throws<ArgumentException>(() => RecommenderSystem.GetRecommendation(questionnaireData, answerData));
        }

        [Test]
        public void TestEmptyAnswer_ReturnsNull()
        {
            var questionnaireData = UtilsForRecommendationTests.GetProjectQuestionnaire();
            var answerData = new AnswerData();

            // Having empty answers is a normal state, where we don't have any recommendation to make.
            var recommendation = RecommenderSystem.GetRecommendation(questionnaireData, answerData);
            Assert.IsNull(recommendation);
        }

        [Test]
        public void TestGetRecommendationForMatchingAnswers_NothingNull()
        {
            var questionnaireData = UtilsForRecommendationTests.GetProjectQuestionnaire();
            var answerData = UtilsForRecommendationTests.BuildAnswerMatching(questionnaireData);
            var recommendation = RecommenderSystem.GetRecommendation(questionnaireData, answerData);
            Assert.NotNull(recommendation);

            foreach (var solution in recommendation.NetcodeOptions)
            {
                if (solution.Solution is PossibleSolution.CustomNetcode or PossibleSolution.NoNetcode) continue;

                UtilsForRecommendationTests.AssertRecommendedSolutionNotNull(solution);
            }

            foreach (var solution in recommendation.ServerArchitectureOptions)
            {
                UtilsForRecommendationTests.AssertRecommendedSolutionNotNull(solution, false);
            }
        }

        [Test]
        public void TestSolutionToPackageViewData_NothingNull()
        {
            var allPackages = RecommenderSystem.GetSolutionsToRecommendedPackageViewData();
            UtilsForRecommendationTests.AssertAllRecommendedPackageNotNull(allPackages);
        }

        [Test]
        public void TestGetRecommendationForMatchingAnswers_OnlyOneMainArchitecturePerCategory()
        {
            var questionnaireData = UtilsForRecommendationTests.GetProjectQuestionnaire();
            var answerData = UtilsForRecommendationTests.BuildAnswerMatching(questionnaireData);
            var recommendation = RecommenderSystem.GetRecommendation(questionnaireData, answerData);
            AssertNoNetcodeIsTheLastNetcodeRecommendation(recommendation);
            Assert.AreEqual(1, recommendation.NetcodeOptions.Count(x => x.RecommendationType == RecommendationType.MainArchitectureChoice));
            Assert.AreEqual(1, recommendation.NetcodeOptions.Count(x => x.Selected));
            Assert.AreEqual(1, recommendation.ServerArchitectureOptions.Count(x => x.RecommendationType == RecommendationType.MainArchitectureChoice));
            Assert.AreEqual(1, recommendation.ServerArchitectureOptions.Count(x => x.Selected));
        }

        [Test]
        public void TestGetSolutionsToRecommendedPackageViewData_AllSelectionsHaveSameCount()
        {
            var allPackages = RecommenderSystem.GetSolutionsToRecommendedPackageViewData();
            
            var expectedCount = 16; // 4 netcode options * 4 hosting options
            Assert.AreEqual(expectedCount, allPackages.Selections.Length);
            
            PossibleSolution[] netcodeSolutions = {PossibleSolution.NGO, PossibleSolution.N4E, PossibleSolution.CustomNetcode, PossibleSolution.NoNetcode};
            PossibleSolution[] hostingSolutions = {PossibleSolution.LS, PossibleSolution.DS, PossibleSolution.CloudCode, PossibleSolution.DA};
            
            var referencePackageCount = allPackages.GetPackagesForSelection(PossibleSolution.NGO, PossibleSolution.LS).Length;
            Assert.True(referencePackageCount > 0);
            
            foreach (var netcode in netcodeSolutions)
            {
                foreach (var hosting in hostingSolutions)
                {
                    var packages = allPackages.GetPackagesForSelection(netcode, hosting);
                    Assert.NotNull(packages);
                    Assert.AreEqual(referencePackageCount, packages.Length);
                }
            }
        }

        [TestCase(PossibleSolution.DS)]
        [TestCase(PossibleSolution.LS)]
        [TestCase(PossibleSolution.DA)]
        [TestCase(PossibleSolution.CloudCode)]
        public void RecommendationData_AllHostingOverridesExistInNetcodeData(PossibleSolution hostingModel)
        {
            var data = RecommenderSystemDataObject.instance.RecommenderSystemData;
            var hostingOverrides = data.SolutionsByType[hostingModel].RecommendedPackages;
            var netcodePackages = data.SolutionsByType[PossibleSolution.NGO].RecommendedPackages;
            
            foreach (var package in hostingOverrides)
            {
                var index = Array.FindIndex(netcodePackages, p => p.PackageId == package.PackageId);
                Assert.True(index > -1, $"Did not find package {package.PackageId} in Netcode packages for {hostingModel}");
            }   
        }

        [TestCase(PossibleSolution.NGO)]
        [TestCase(PossibleSolution.N4E)]
        [TestCase(PossibleSolution.CustomNetcode)]
        [TestCase(PossibleSolution.NoNetcode)]
        public void RecommendationData_NetcodeSolutionsHaveRecommendationDataForAllPackages(PossibleSolution netcode)
        {
            var data = RecommenderSystemDataObject.instance.RecommenderSystemData;
            var packagesForNetcode = data.SolutionsByType[netcode].RecommendedPackages;
            var solutionPackages = data.RecommendedSolutions.Where(e => e.MainPackageId != null).Select(e => e.MainPackageId).ToArray();
            
            foreach (var package in data.Packages)
            {
                if(solutionPackages.Contains(package.Id))
                    continue;
                
                var index = Array.FindIndex(packagesForNetcode, p => p.PackageId == package.Id);
                Assert.True(index > -1, $"Did not find package {package.Id} in packages of {netcode}");
            }
        }
        
        // Regression test: selecting back and forth between netcode options should not change whether some solutions
        // are compatible or not
        [Test]
        public void TestAdaptRecommendationToNetcodeSelection_SelectBackAndForthDoesNotChangeRecommendations()
        {
            var recommendation = UtilsForRecommendationTests.ComputeRecommendationForPreset(Preset.Adventure);
            var selectedNetcode = RecommendationUtils.GetSelectedNetcode(recommendation);
            Assert.AreEqual(PossibleSolution.NGO, selectedNetcode.Solution, "Wrong test setup: expected NGO to be selected");
            AssertCompatibilityInView(true, PossibleSolution.DA, recommendation.ServerArchitectureOptions, "Wrong test setup: expected DA to be compatible");

            UtilsForRecommendationTests.SimulateSelectionChange(PossibleSolution.N4E, recommendation.NetcodeOptions);
            RecommenderSystem.AdaptRecommendationToNetcodeSelection(recommendation);
            Assert.AreEqual(PossibleSolution.N4E, RecommendationUtils.GetSelectedNetcode(recommendation).Solution, "Wrong test setup: expected N4E to be selected");
            AssertCompatibilityInView(false, PossibleSolution.DA, recommendation.ServerArchitectureOptions,"Wrong test setup: expected DA to be incompatible with N4E");

            UtilsForRecommendationTests.SimulateSelectionChange(PossibleSolution.NGO, recommendation.NetcodeOptions);
            RecommenderSystem.AdaptRecommendationToNetcodeSelection(recommendation);
            Assert.AreEqual(PossibleSolution.NGO, RecommendationUtils.GetSelectedNetcode(recommendation).Solution, "Wrong test setup: expected NGO to be selected");
            AssertCompatibilityInView(true, PossibleSolution.DA, recommendation.ServerArchitectureOptions, "Fail: expected DA to still be compatible with NGO after selecting other solution");
        }

        [Test]
        public void TestAdaptIncompatibility_AllValuesMatch()
        {
            bool ngoDaCompatibleBefore = AreCompatible(PossibleSolution.NGO, PossibleSolution.DA);
            Assert.True(ngoDaCompatibleBefore, "Wrong test setup: expected NGO to be compatible with DA");
            
            var ngoSolBefore = GetSolution(PossibleSolution.NGO);
            Assert.AreEqual(-1, Array.FindIndex(ngoSolBefore.IncompatibleSolutions, s => s.Solution == PossibleSolution.DA), "Wrong test setup: expected DA to be in the incompatible list of NGO");
            
            RecommenderSystemDataObject.instance.RecommenderSystemData.UpdateIncompatibility(PossibleSolution.NGO, 
                PossibleSolution.DA, false, "test");
            
            bool ngoDaCompatibleAfter = AreCompatible(PossibleSolution.NGO, PossibleSolution.DA);
            Assert.False(ngoDaCompatibleAfter);
            
            var ngoSolAfter = GetSolution(PossibleSolution.NGO);
            Assert.AreNotEqual(-1, Array.FindIndex(ngoSolAfter.IncompatibleSolutions, s => s.Solution == PossibleSolution.DA));
            
            RecommenderSystemDataObject.instance.RecommenderSystemData.UpdateIncompatibility(PossibleSolution.NGO, PossibleSolution.DA, true);
          
            bool ngoDaCompatibleEnd = AreCompatible(PossibleSolution.NGO, PossibleSolution.DA);
            Assert.True(ngoDaCompatibleEnd, "Wrong test setup: expected NGO to be compatible with DA");
            
            var ngoSolEnd = GetSolution(PossibleSolution.NGO);
            Assert.AreEqual(-1, Array.FindIndex(ngoSolEnd.IncompatibleSolutions, s => s.Solution == PossibleSolution.DA));
        }
        
        static bool AreCompatible(PossibleSolution netcode, PossibleSolution hostingModel)
            => RecommenderSystemDataObject.instance.RecommenderSystemData.IsHostingModelCompatibleWithNetcode(
                PossibleSolution.NGO, PossibleSolution.DA, out _);
        
        static RecommendedSolution GetSolution(PossibleSolution type)
            => RecommenderSystemDataObject.instance.RecommenderSystemData.SolutionsByType[type];
        
        static void AssertNoNetcodeIsTheLastNetcodeRecommendation(RecommendationViewData recommendation)
        {
            var netcodeSolutionCount = recommendation.NetcodeOptions.Length;
            Assert.AreEqual(4, netcodeSolutionCount);
            var lastSolution = recommendation.NetcodeOptions[netcodeSolutionCount - 1];
            Assert.AreEqual(k_NoNetcodeTitle, lastSolution.Title);
            Assert.False(lastSolution.Selected);
        }

        static void AssertCompatibilityInView(bool expectedCompatible, PossibleSolution hostingModel, RecommendedSolutionViewData[] hostingSolutions, string msg)
        {
            foreach (var hosting in hostingSolutions)
            {
                if(hosting.Solution == hostingModel)
                {
                    if(expectedCompatible)
                        Assert.AreNotEqual(RecommendationType.Incompatible, hosting.RecommendationType, msg);
                    else
                        Assert.AreEqual(RecommendationType.Incompatible, hosting.RecommendationType, msg);
                }
            }
        }
    }
}
