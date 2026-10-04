using System;
using NUnit.Framework;
using Unity.Multiplayer.Center.Common;
using Unity.Multiplayer.Center.Questionnaire;
using UnityEngine;

namespace Unity.MultiplayerCenterTests
{
    [TestFixture]
    class LogicTests
    {
        [Test]
        public void TryGetQuestionByQuestionId_IdExists_Found()
        {
            var questionnaire = UtilsForRecommendationTests.GetProjectQuestionnaire();
            var success = Logic.TryGetQuestionByQuestionId(questionnaire, questionnaire.Questions[1].Id, out var question);
            Assert.True(success);
            Assert.NotNull(question);
        }
        
        [Test]
        public void TryGetQuestionByQuestionId_StringIdExists_Found()
        {
            var questionnaire = UtilsForRecommendationTests.GetProjectQuestionnaire();
            var success = Logic.TryGetQuestionByQuestionId(questionnaire, "Pace", out var question);
            Assert.True(success);
            Assert.NotNull(question);
        }
        
        [Test]
        public void TryGetQuestionByQuestionId_IdDoesNotExist_NotFound()
        {
            var questionnaire = UtilsForRecommendationTests.GetProjectQuestionnaire();
            var success = Logic.TryGetQuestionByQuestionId(questionnaire, "nonexistent", out var question);
            Assert.False(success);
            Assert.Null(question);
        }

        [Test]
        public void TryGetAnswerByQuestionId_IdExists_Found()
        {
            var questionnaire = UtilsForRecommendationTests.GetProjectQuestionnaire();
            var userAnswers = UtilsForRecommendationTests.BuildAnswerMatching(questionnaire);
            var success = Logic.TryGetAnswerByQuestionId(userAnswers, "PlayerCount", out var answer);
            Assert.True(success);
            Assert.NotNull(answer);
            Assert.AreEqual(1, answer.Answers.Count);
        }
        
        [Test]
        public void TryGetAnswerByQuestionId_IdDoesNotExist_NotFound()
        {
            var questionnaire = UtilsForRecommendationTests.GetProjectQuestionnaire();
            var userAnswers = UtilsForRecommendationTests.BuildAnswerMatching(questionnaire);
            var success = Logic.TryGetAnswerByQuestionId(userAnswers, "nonexistent", out var answer);
            Assert.False(success);
            Assert.Null(answer);
        }
        
        [Test]
        public void TestApplyPresetToAnswerData_WhenPlayerCountIsSet_PlayerCountStays()
        {
            var questionnaire = UtilsForRecommendationTests.GetProjectQuestionnaire();
            var userAnswers = UtilsForRecommendationTests.BuildAnswerMatching(questionnaire);
            
            Logic.TryGetAnswerByQuestionId(userAnswers, "PlayerCount", out var answer);
            var chosenPlayerCount = Int32.Parse(answer.Answers[0]);
            
            // No default answer, otherwise this test could succeed even if the logic is wrong
            Assert.AreNotEqual(0, chosenPlayerCount);
            Assert.AreNotEqual(2, chosenPlayerCount);

            var (newAnswer, newReco) = Logic.ApplyPresetToAnswerData(userAnswers, Preset.Strategy, questionnaire);
            Assert.NotNull(newAnswer);
            Logic.TryGetAnswerByQuestionId(newAnswer, "PlayerCount", out var newAnsweredQuestion);
            var newChosenPlayerCount = Int32.Parse(newAnsweredQuestion.Answers[0]);
            Assert.AreEqual(chosenPlayerCount, newChosenPlayerCount);
            Assert.NotNull(newReco);
        }
        
        [TestCase("1.2", "1.3", true)]
        [TestCase("3.2", "1.3", false)]
        [TestCase("1.21", "1.2", false)]
        [TestCase("1.2", "1.2", false)]
        [TestCase("0.2", "1.3", true)]
        [TestCase("1.3.12", "1.3.13", true)]
        [TestCase("1.3.11", "0.23", false)]
        [TestCase("3.3.33", "3.3.33", false)]
        [TestCase("3.3.3", "3.3.33", true)]
        public void TestIsVersionLower_ReturnsCorrectResult(string versionToTest, string currentVersion, params bool[] expected)
        {
            var result = Logic.IsVersionLower(versionToTest, currentVersion);
            Assert.AreEqual(expected[0], result);
        }
    }
}
