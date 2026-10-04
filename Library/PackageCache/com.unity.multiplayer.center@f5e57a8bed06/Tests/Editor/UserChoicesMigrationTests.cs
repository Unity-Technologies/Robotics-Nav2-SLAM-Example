using System;
using System.Collections.Generic;
using NUnit.Framework;
using Unity.Multiplayer.Center.Common;
using Unity.Multiplayer.Center.Questionnaire;
using UnityEngine;

namespace Unity.MultiplayerCenterTests
{
    [TestFixture]
    internal class UserChoicesMigrationTests
    {
        [OneTimeSetUp]
        public void OneTimeSetup()
        {
            // Copy user choices to temp file to restore after tests.
            UtilsForMultiplayerCenterTests.CopyUserChoicesToTempFile();
        }

        [OneTimeTearDown]
        public void OneTimeTearDown()
        {
            // Restore user choices after tests.
            UtilsForMultiplayerCenterTests.RestoreUserChoicesFromTempFile();
        }
        
        [Test]
        public void TestMigration_Pre1_2To1_2_RemovesCompetitivenessButNothingElse()
        {
            var questionnaire = UtilsForRecommendationTests.GetProjectQuestionnaire();
            questionnaire.Version = "1.2";
            var userAnswers = new AnswerData()
            {
                Answers = new List<AnsweredQuestion>()
                {
                    new () { QuestionId = "CostSensitivity", Answers = new List<string>() {"BestMargin"}},
                    new () { QuestionId = "Pace", Answers = new List<string>() {"Slow"}},
                    new () { QuestionId = "Competitiveness", Answers = new List<string>() {"Competitive"}},
                    new () { QuestionId = "PlayerCount", Answers = new List<string>() {"2"}}
                }
            };
            UserChoicesObject.instance.UserAnswers = userAnswers;
            UserChoicesObject.instance.QuestionnaireVersion = null;
            
            var foundCompetitive = Logic.TryGetAnswerByQuestionId(userAnswers, "Competitiveness", out var answer);
            Assert.True(foundCompetitive);
            Assert.NotNull(answer);
            
            var errorsBefore = Logic.ValidateAnswers(questionnaire, UserChoicesObject.instance.UserAnswers);
            Assert.IsNotEmpty(errorsBefore);
            
            Logic.MigrateUserChoices(questionnaire, UserChoicesObject.instance);
            
            var foundCompetitiveAfterMigration = Logic.TryGetAnswerByQuestionId(userAnswers, "Competitiveness", out var answerAfterMigration);
            Assert.False(foundCompetitiveAfterMigration);
            Assert.Null(answerAfterMigration);
            
            Assert.NotNull(UserChoicesObject.instance.QuestionnaireVersion);
            
            Assert.AreEqual(3,  UserChoicesObject.instance.UserAnswers.Answers.Count);
            
            var errors = Logic.ValidateAnswers(questionnaire, UserChoicesObject.instance.UserAnswers);
            Assert.IsEmpty(errors);
        }
        
        [Test]
        public void TestMigration_1_2To1_3_ChangesMediumPaceToSlow()
        {
            var questionnaire = UtilsForRecommendationTests.GetProjectQuestionnaire();
            questionnaire.Version = "1.3";
            var userAnswers = new AnswerData()
            {
                Answers = new List<AnsweredQuestion>()
                {
                    new () { QuestionId = "CostSensitivity", Answers = new List<string>() {"BestMargin"}},
                    new () { QuestionId = "Pace", Answers = new List<string>() {"Medium"}},
                    new () { QuestionId = "PlayerCount", Answers = new List<string>() {"2"}}
                }
            };
            UserChoicesObject.instance.UserAnswers = userAnswers;
            UserChoicesObject.instance.QuestionnaireVersion = "1.2";
            
            var foundPace = Logic.TryGetAnswerByQuestionId(userAnswers, "Pace", out var answer);
            Assert.True(foundPace);
            Assert.NotNull(answer);
            
            Logic.MigrateUserChoices(questionnaire, UserChoicesObject.instance);
            
            var foundPaceAfterMigration = Logic.TryGetAnswerByQuestionId(userAnswers, "Pace", out var answerAfterMigration);
            Assert.True(foundPaceAfterMigration);
            Assert.NotNull(answerAfterMigration);
            Assert.AreEqual("Slow", answerAfterMigration.Answers[0]);
            
            var errors = Logic.ValidateAnswers(questionnaire, UserChoicesObject.instance.UserAnswers);
            Assert.IsEmpty(errors);
        }
        
        
        [Test]
        public void TestMigration_SameVersion_RemovesNothing()
        {
            var questionnaire = UtilsForRecommendationTests.GetProjectQuestionnaire();
            var userAnswers = new AnswerData()
            {
                Answers = new List<AnsweredQuestion>()
                {
                    new () { QuestionId = "CostSensitivity", Answers = new List<string>() {"BestMargin"}},
                    new () { QuestionId = "Pace", Answers = new List<string>() {"Slow"}},
                    new () { QuestionId = "PlayerCount", Answers = new List<string>() {"2"}}
                }
            };
            UserChoicesObject.instance.UserAnswers = userAnswers;
            UserChoicesObject.instance.QuestionnaireVersion = "1.3";
            
            Logic.MigrateUserChoices(questionnaire, UserChoicesObject.instance);

            Assert.AreEqual("1.3", UserChoicesObject.instance.QuestionnaireVersion);
            
            Assert.AreEqual(3,  UserChoicesObject.instance.UserAnswers.Answers.Count);
            
            var errors = Logic.ValidateAnswers(questionnaire, UserChoicesObject.instance.UserAnswers);
            Assert.IsEmpty(errors);
        }
    }
}
