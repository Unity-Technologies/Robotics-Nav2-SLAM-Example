using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Analytics;

namespace Unity.Multiplayer.Center.Analytics
{
    /// <summary>
    /// Does the same as the MultiplayerCenterAnalytics, but logs the events to the console instead of sending them.
    /// It is useful to debug fast, without the EditorAnalytics Debugger package (but it does not replace it).
    /// </summary>
    internal class DebugAnalytics : MultiplayerCenterAnalytics
    {
        public DebugAnalytics(string questionnaireVersion, IReadOnlyDictionary<string, string> questionDisplayNames,
            IReadOnlyDictionary<string,string> answerDisplayNames) 
            : base(questionnaireVersion, questionDisplayNames, answerDisplayNames) { }

        protected override void SendAnalytic(IAnalytic analytic)
        {
            analytic.TryGatherData(out var data, out var _);
            switch (data)
            {
                case InstallData installData:
                    Debug.Log($"Event: {analytic.GetType()} - Data: {ToString(installData)}");
                    break;
                case RecommendationData recommendationData:
                    Debug.Log($"Event: {analytic.GetType()} - Data: {ToString(recommendationData)}");
                    break;
                case InteractionData interactionEventAnalytic:
                    Debug.Log($"Event: {analytic.GetType()} - Data: {ToString(interactionEventAnalytic)}");
                    break;
                default:
                    Debug.Log($"Unknown event: {analytic.GetType()} - Data: {data}");
                    break;
            }
        }
        
        static string ToString(GameSpec p) => $"GameSpec [{p.QuestionText} ->  {p.AnswerText}]";
        
        static string ToString(Package p) => $"Package [{p.PackageId} - Selected {p.SelectedForInstall} - Reco {p.IsRecommended} - Inst {p.IsAlreadyInstalled}]";
        
        static string ToString(InstallData data)
        {
            var packageStrings = new List<string>(data.Packages.Length);
            foreach (var package in data.Packages)
            {
                packageStrings.Add(ToString(package));
            }
            return $"{data.PresetName} - Packages [{data.Packages.Length}] packages: \n{string.Join("\n", packageStrings)}";
        }

        static string ToString(RecommendationData data)
        {
            var gameSpecStrings = new List<string>(data.GameSpecs.Length);
            foreach (var gameSpec in data.GameSpecs)
            {
                gameSpecStrings.Add(ToString(gameSpec));
            }
            return $"{data.PresetName} - GameSpecs [{data.GameSpecs.Length}] gamespecs: \n{string.Join("\n", gameSpecStrings)}";
        }

        static string ToString(InteractionData data) => $"{data.SectionId}({data.TargetPackageId}) - {data.Type} - {data.DisplayName}";
    }
}
