using System;
using System.Collections.Generic;
using Unity.Multiplayer.Center.Common;
using Unity.Multiplayer.Center.Common.Analytics;
using UnityEditor;
using UnityEngine;
using UnityEngine.Analytics;

namespace Unity.Multiplayer.Center.Analytics
{
    /// <summary>
    /// The interface for the Multiplayer Center Analytics provider (only one functional implementation, but the
    /// interface is needed for testing purposes)
    /// </summary>
    internal interface IMultiplayerCenterAnalytics
    {
        void SendInstallationEvent(AnswerData data, Preset preset, Package[] packages,
            string hostingModelName, bool hostingModelRecommended, string netcodeSolutionName, bool netcodeSolutionRecommended);
        void SendRecommendationEvent(AnswerData data, Preset preset);
        void SendGettingStartedInteractionEvent(string targetPackageId, string sectionId, InteractionDataType type, string displayName);
    }

    /// <summary>
    /// The concrete implementation of the multiplayer center analytics provider.
    /// It convert 
    /// </summary>
    internal class MultiplayerCenterAnalytics : IMultiplayerCenterAnalytics
    {
        const string k_VendorKey = "unity.multiplayer.center";
        const string k_InstallationEventName = "multiplayer_center_onInstallClicked";
        const string k_RecommendationEventName = "multiplayer_center_onRecommendation";
        const string k_GetStartedInteractionEventName = "multiplayer_center_onGetStartedInteraction";
        
        readonly string m_QuestionnaireVersion;
        readonly IReadOnlyDictionary<string, string> m_AnswerIdToAnswerName;
        readonly IReadOnlyDictionary<string, string> m_QuestionIdToQuestionName;
        readonly string[] m_PresetFullNames = AnalyticsUtils.GetPresetFullNames();
        string PresetName(Preset v) => m_PresetFullNames[(int)v];

        GameSpec[] FillGameSpecs(AnswerData data)
        {
            return AnalyticsUtils.ToGameSpecs(data, m_AnswerIdToAnswerName, m_QuestionIdToQuestionName);
        }
        
        protected virtual void SendAnalytic(IAnalytic analytic)
        {
            EditorAnalytics.SendAnalytic(analytic);
        } 

        public MultiplayerCenterAnalytics(string questionnaireVersion, IReadOnlyDictionary<string, string> questionDisplayNames,
            IReadOnlyDictionary<string, string> answerDisplayNames)
        {
            m_QuestionnaireVersion = questionnaireVersion;
            m_QuestionIdToQuestionName = questionDisplayNames;
            m_AnswerIdToAnswerName = answerDisplayNames;
        }
        
        public void SendGettingStartedInteractionEvent(string targetPackageId, string sectionId, InteractionDataType type, string displayName)
        {
            var analytic = new GetStartedInteractionEventAnalytic(sectionId, type, displayName, targetPackageId);
            SendAnalytic(analytic);
        }

        public void SendInstallationEvent(AnswerData data, Preset preset, Package[] packages,
            string hostingModelName, bool hostingModelRecommended, string netcodeSolutionName, bool netcodeSolutionRecommended)
        {
            var analytic = new InstallationEventAnalytic(new InstallData()
            {
                Preset = (int)preset,
                PresetName = PresetName(preset),
                QuestionnaireVersion = m_QuestionnaireVersion,
                GamesSpecs = FillGameSpecs(data),
                Packages = packages,
                hostingModelName = hostingModelName,
                hostingModelRecommended = hostingModelRecommended,
                netcodeSolutionName = netcodeSolutionName,
                netcodeSolutionRecommended = netcodeSolutionRecommended
            });
            SendAnalytic(analytic);
        }

        public void SendRecommendationEvent(AnswerData data, Preset preset)
        {
            var analytic = new RecommendationEventAnalytic(new RecommendationData()
            {
                Preset = (int)preset,
                PresetName = PresetName(preset),
                QuestionnaireVersion = m_QuestionnaireVersion,
                GameSpecs = FillGameSpecs(data)
            });
            SendAnalytic(analytic);
        }

        [AnalyticInfo(eventName: k_InstallationEventName, vendorKey: k_VendorKey)]
        private class InstallationEventAnalytic : IAnalytic
        {
            InstallData m_Data;

            public InstallationEventAnalytic(InstallData data)
            {
                m_Data = data;
            }
            
            /// <inheritdoc />
            public bool TryGatherData(out IAnalytic.IData data, out Exception error)
            {
                data = m_Data;
                error = null;
                return true;
            }
        }

        [AnalyticInfo(eventName: k_RecommendationEventName, vendorKey: k_VendorKey)]
        private class RecommendationEventAnalytic : IAnalytic
        {
            RecommendationData m_Data;

            public RecommendationEventAnalytic(RecommendationData data)
            {
                m_Data = data;
            }
            
            public bool TryGatherData(out IAnalytic.IData data, out Exception error)
            {
                data = m_Data;
                error = null;
                return true;
            }
        }

        [AnalyticInfo(eventName: k_GetStartedInteractionEventName, vendorKey: k_VendorKey)]
        private class GetStartedInteractionEventAnalytic : IAnalytic
        {
            InteractionData m_Data;
            
            public GetStartedInteractionEventAnalytic(string sectionId, InteractionDataType type, string displayName, string targetPackageId)
            {
                m_Data = new InteractionData()
                {
                    SectionId = sectionId,
                    Type = type,
                    DisplayName = displayName,
                    TargetPackageId = targetPackageId
                };
            }
            
            /// <inheritdoc />
            public bool TryGatherData(out IAnalytic.IData data, out Exception error)
            { 
                data = m_Data;
                error = null;
                return true;
            }
        }
    }
}
