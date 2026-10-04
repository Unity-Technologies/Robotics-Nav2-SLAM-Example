using System;
using Unity.Multiplayer.Center.Questionnaire;

namespace Unity.Multiplayer.Center.Analytics
{
    internal static class MultiplayerCenterAnalyticsFactory
    {
        public static IMultiplayerCenterAnalytics Create()
        {
            var questionnaire = QuestionnaireObject.instance;
            var questionnaireVersion = questionnaire.Questionnaire.Version;
            var questionDisplayNames = AnalyticsUtils.GetQuestionDisplayNames(questionnaire.Questionnaire);
            var answerDisplayNames = AnalyticsUtils.GetAnswerDisplayNames(questionnaire.Questionnaire);

            // Uncomment this line to use the DebugAnalytics class instead of the MultiplayerCenterAnalytics class
            // return new DebugAnalytics(questionnaireVersion, questionDisplayNames, answerDisplayNames);
            
            return new MultiplayerCenterAnalytics(questionnaireVersion, questionDisplayNames, answerDisplayNames);
        }
    }
}
