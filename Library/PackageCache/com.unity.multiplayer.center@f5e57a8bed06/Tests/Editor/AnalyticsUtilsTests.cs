using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using Unity.Multiplayer.Center.Analytics;
using Unity.Multiplayer.Center.Common;
using Unity.Multiplayer.Center.Recommendations;

namespace Unity.MultiplayerCenterTests
{
    [TestFixture]
    internal class AnalyticsUtilsTests
    {
        [Test]
        public void AnalyticsUtils_GetQuestionDisplayNames_RightCount()
        {
            var questionnaire = UtilsForRecommendationTests.GetProjectQuestionnaire();
            var questionDisplayNames = AnalyticsUtils.GetQuestionDisplayNames(questionnaire);
            Assert.AreEqual(questionDisplayNames.Count, questionnaire.Questions.Length);
        }
        
        /// <summary>
        /// Note: this test checks exact values and has to be adapted in case those display names change.
        /// Why do that? To force the developers to think about the impact on the analytics!
        /// Contact the engine-data team and product.
        /// </summary>
        [Test]
        public void AnalyticsUtils_GetQuestionDisplayNames_RightValuesForSelectedQuestions()
        {
            var questionnaire = UtilsForRecommendationTests.GetProjectQuestionnaire();
            var questionDisplayNames = AnalyticsUtils.GetQuestionDisplayNames(questionnaire);
            Assert.AreEqual("Number of Players per Session", questionDisplayNames["PlayerCount"]);
            Assert.AreEqual("Gameplay Pace", questionDisplayNames["Pace"]);
            Assert.AreEqual("Cheating / Modding Prevention", questionDisplayNames["Cheating"]);
            Assert.AreEqual("Cost Sensitivity", questionDisplayNames["CostSensitivity"]);
            Assert.AreEqual("Netcode Architecture", questionDisplayNames["NetcodeArchitecture"]);
        }
        
        [Test]
        public void AnalyticsUtils_GetAnswerDisplayNames_RightCount()
        {
            var questionnaire = UtilsForRecommendationTests.GetProjectQuestionnaire();
            var answerDisplayNames = AnalyticsUtils.GetAnswerDisplayNames(questionnaire);
            
            // Note: 2 birds one stone: this test also nicely checks if we have duplicates with question ids
            var expectedCount = questionnaire.Questions.Sum(question => question.Choices.Length);
            Assert.AreEqual(answerDisplayNames.Count, expectedCount);
        }
        
        [Test]
        public void AnalyticsUtils_GetPresetFullNames_RightCountAndIntValuesDidNotChange()
        {
            var presetFullNames = AnalyticsUtils.GetPresetFullNames();
            Assert.AreEqual(12, presetFullNames.Length);
        }
        
        /// <summary>
        /// Note: this test checks exact values and has to be adapted in case those display names change.
        /// Why do that? To force the developers to think about the impact on the analytics!
        /// Contact the engine-data team and product.
        /// </summary>
        [Test]
        public void AnalyticsUtils_GetPresetFullNames_RightValues()
        {
            var presetFullNames = AnalyticsUtils.GetPresetFullNames();
            Assert.AreEqual("-", presetFullNames[0]);
            Assert.AreEqual("Adventure", presetFullNames[1]);
            Assert.AreEqual("Shooter, Battle Royale, Battle Arena", presetFullNames[2]);
            Assert.AreEqual("Racing", presetFullNames[3]);
            Assert.AreEqual("Card Battle, Turn-based, Tabletop", presetFullNames[4]);
            Assert.AreEqual("Simulation", presetFullNames[5]);
            Assert.AreEqual("Strategy", presetFullNames[6]);
            Assert.AreEqual("Sports", presetFullNames[7]);
            Assert.AreEqual("Role-Playing, MMO", presetFullNames[8]);
            Assert.AreEqual("Async, Idle, Hyper Casual, Puzzle", presetFullNames[9]);
            Assert.AreEqual("Fighting", presetFullNames[10]);
            Assert.AreEqual("Arcade, Platformer, Sandbox", presetFullNames[11]);
        }
        
        [Test]
        public void AnalyticsUtils_ToGameSpecs_AllIdsAreInTheGameSpecs()
        {
            var questionnaire = UtilsForRecommendationTests.GetProjectQuestionnaire();
            var answerDisplayNames = AnalyticsUtils.GetAnswerDisplayNames(questionnaire);
            var answerData = UtilsForRecommendationTests.BuildAnswerMatching(questionnaire);
            var questionIdToQuestionName = AnalyticsUtils.GetQuestionDisplayNames(questionnaire);
            var gameSpecs = AnalyticsUtils.ToGameSpecs(answerData, answerDisplayNames, questionIdToQuestionName);
            Assert.AreEqual(answerData.Answers.Count, gameSpecs.Length);
            
            var questionIdsFromAnswerData = answerData.Answers.Select(answer => answer.QuestionId).ToArray();
            var questionIdsFromGameSpecs = gameSpecs.Select(spec => spec.QuestionId).ToArray();
            CollectionAssert.AreEquivalent(questionIdsFromAnswerData, questionIdsFromGameSpecs);
        }

        [Test]
        public void AnalyticsUtils_ToGameSpecs_ValueCheck()
        {
            var answerDisplayNames = new Dictionary<string, string>(){
                {"A1", "Display Name A1"},
                {"A2", "Display Name A2"},
                {"A3", "Display Name A3"},
                {"A4", "Display Name A4"}
            };
            var questionIdToQuestionName = new Dictionary<string, string>()
            {
                {"Q1", "Display Name Q1"},
                {"Q2", "Display Name Q2"}
            };
            var answerData = new AnswerData()
            {
                Answers = new List<AnsweredQuestion>()
                {
                    new() {QuestionId = "Q2", Answers = new List<string>() {"A3"}},
                    new() {QuestionId = "Q1", Answers = new List<string>() {"A1"}}
                }
            };
            var gameSpecs = AnalyticsUtils.ToGameSpecs(answerData, answerDisplayNames, questionIdToQuestionName);

            var expectedGameSpecs = new GameSpec[]
            {
                new (){ QuestionId = "Q2", QuestionText = "Display Name Q2", AcceptsMultipleAnswers = false, AnswerId = "A3", AnswerText = "Display Name A3"},
                new (){ QuestionId = "Q1", QuestionText = "Display Name Q1", AcceptsMultipleAnswers = false, AnswerId = "A1", AnswerText = "Display Name A1"},
            };
            CollectionAssert.AreEquivalent(expectedGameSpecs, gameSpecs);
        }

        [Test]
        public void AnalyticsUtils_ToGameSpecs_NoEmptyString()
        {
            GetAnswersWithMatchingAnalyticsData(out var _, out var gameSpecs);
            foreach (var spec in gameSpecs)
            {
                Assert.False(string.IsNullOrEmpty(spec.QuestionId), spec.QuestionId);
                Assert.False(string.IsNullOrEmpty(spec.QuestionText), spec.QuestionText);
                Assert.False(string.IsNullOrEmpty(spec.AnswerId), spec.AnswerId);
                Assert.False(string.IsNullOrEmpty(spec.AnswerText), spec.AnswerText);
            }
        }
        
        [Test]
        public void AnalyticsUtils_AssumptionTest_HardCodedNumNetcodePackageMatchesRecommendations()
        {
            // note that the amount of possible netcode does not change with the answer data.
            var someRecommendation = UtilsForRecommendationTests.GetSomeRecommendation();
            var netcodePackageCount = someRecommendation.NetcodeOptions.Count(e
                => !string.IsNullOrEmpty(e.MainPackage?.PackageId));
            
            Assert.AreEqual(netcodePackageCount, AnalyticsUtils.NumNetcodePackage);
        }
        
        [Test]
        public void AnalyticsUtils_AssumptionTest_HardCodedNumHostingPackageMatchesRecommendations()
        {
            // note that the amount of possible netcode does not change with the answer data.
            var someRecommendation = UtilsForRecommendationTests.GetSomeRecommendation();
            var hostingPackageCount = someRecommendation.ServerArchitectureOptions.Count(e
                => !string.IsNullOrEmpty(e.MainPackage?.PackageId));
            
            Assert.AreEqual(hostingPackageCount, AnalyticsUtils.NumHostingPackages);
        }

        [Test]
        public void AnalyticsUtils_GetPackagesWithAnalyticsFormat_NetcodeValuesMakeSense()
        {
            const string ngo = "com.unity.netcode.gameobjects";
            const string n4e = "com.unity.netcode";
            const string tp = "com.unity.transport";
            
            var someRecommendation = UtilsForRecommendationTests.GetSomeRecommendation();
            var packageViews = RecommenderSystem.GetSolutionsToRecommendedPackageViewData();
            var packagesAnalyticsFormat = AnalyticsUtils.GetPackagesWithAnalyticsFormat(someRecommendation, packageViews);

            // all netcode packages are in the array
            Assert.AreEqual(1,packagesAnalyticsFormat.Count(e => e.PackageId == ngo) );
            Assert.AreEqual(1, packagesAnalyticsFormat.Count(e => e.PackageId == n4e));
            Assert.AreEqual(1, packagesAnalyticsFormat.Count(e => e.PackageId == tp));
            
            var netcodePackages = packagesAnalyticsFormat.Where(e => e.PackageId is ngo or n4e or tp).ToArray();
            Assert.AreEqual(3, netcodePackages.Count()); // exactly 3 matches
            Assert.AreEqual(1, netcodePackages.Count(e => e.SelectedForInstall)); // 1 selected
            Assert.AreEqual(1, netcodePackages.Count(e => e.IsRecommended)); // 1 recommended
            
            // Since we only look at the project direct dependencies, 1 netcode max can be installed in a normal setup
            // edge case would be: some installed ngo/n4e and manually added Transport to the manifest
            Assert.True(netcodePackages.Count(e => e.IsAlreadyInstalled) <= 1);
        }

        [Test]
        public void AnalyticsUtils_GetPackagesWithAnalyticsFormat_AllPackagesAreIn()
        {
            var someRecommendation = UtilsForRecommendationTests.GetSomeRecommendation();
            var packageViews = RecommenderSystem.GetSolutionsToRecommendedPackageViewData();
            var packagesAnalyticsFormat = AnalyticsUtils.GetPackagesWithAnalyticsFormat(someRecommendation, packageViews);
            
            var allPackages = RecommenderSystemDataObject.instance.RecommenderSystemData.PackageDetailsById.Keys;
            
            foreach (var id in allPackages)
            {
                Assert.AreEqual(1, packagesAnalyticsFormat.Count(e => e.PackageId == id), id);
            }
        }
        
        static void GetAnswersWithMatchingAnalyticsData(out AnswerData answerData, out GameSpec[] gameSpecs)
        {
            var questionnaire = UtilsForRecommendationTests.GetProjectQuestionnaire();
            var answerDisplayNames = AnalyticsUtils.GetAnswerDisplayNames(questionnaire);
            answerData = UtilsForRecommendationTests.BuildAnswerMatching(questionnaire);
            var questionIdToQuestionName = AnalyticsUtils.GetQuestionDisplayNames(questionnaire);
            gameSpecs = AnalyticsUtils.ToGameSpecs(answerData, answerDisplayNames, questionIdToQuestionName);
        }
    }
}
