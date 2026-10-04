using System.Linq;
using NUnit.Framework;
using Unity.Multiplayer.Center.Recommendations;

namespace Unity.MultiplayerCenterTests
{
    /// <summary>
    /// Cheap checks on some user visible text.
    /// </summary>
    [TestFixture]
    internal class UserVisibleTextTests
    {
        static readonly string[] k_Verbs = {"is", "offers", "can", "costs", "would", "should", "works", "tends", "enables"};
        
        [Test]
        public void AllScoreImpacts_ShouldHaveANonEmptyReason()
        {
            var questionnaireData = UtilsForRecommendationTests.GetProjectQuestionnaire();
            foreach (var question in questionnaireData.Questions)
            {
                foreach (var answer in question.Choices)
                {
                    for (var index = 0; index < answer.ScoreImpacts.Length; index++)
                    {
                        var scoreImpact = answer.ScoreImpacts[index];
                        Assert.False(string.IsNullOrEmpty(scoreImpact.Comment), 
                            $"Comment is empty for question {question.Id} answer {answer.Id} impact at index {index}");
                    }
                }
            }
        }
        
        [Test]
        public void AllScoreImpacts_StartWithAVerbAndDoNotEndWithADot()
        {
            var questionnaireData = UtilsForRecommendationTests.GetProjectQuestionnaire();
            foreach (var question in questionnaireData.Questions)
            {
                foreach (var answer in question.Choices)
                {
                    for (var index = 0; index < answer.ScoreImpacts.Length; index++)
                    {
                        var comment = answer.ScoreImpacts[index].Comment;
                        var firstWord = comment.Split(' ')[0];
                        CollectionAssert.Contains(k_Verbs, firstWord, 
                            $"Comment '{comment}' does not start with a verb for question {question.Id} answer {answer.Id} impact at index {index}");
                        Assert.False(comment.EndsWith("."), 
                            $"Comment '{comment}' should not end with a dot for question {question.Id} answer {answer.Id} impact at index {index}");
                    }
                }
            }
        }
        
        [Test]
        public void AllSolutionsData_DoNotHaveAVerbBeforeDynamicText()
        {
            var data = RecommenderSystemDataObject.instance.RecommenderSystemData;
            const string dynamicKeyword = Scoring.DynamicKeyword;
            foreach (var solution in data.RecommendedSolutions)
            {
                Assert.True(solution.ShortDescription.Contains(dynamicKeyword), 
                    $"Solution {solution.Type} description does not contain dynamic text '{dynamicKeyword}'");
                var wordBeforeDynamic = solution.ShortDescription.Split(dynamicKeyword)[0].Split(' ').Last();
                Assert.False(k_Verbs.Contains(wordBeforeDynamic), 
                    $"Solution {solution.Type} description starts with a verb '{wordBeforeDynamic}' before dynamic text '{dynamicKeyword}'");
            }
        }

    }
}
