using System;
using Unity.Multiplayer.Center.Common.Analytics;
using UnityEngine;

namespace Unity.Multiplayer.Center.Analytics
{
    /// <summary>
    /// The concrete implementation of the IOnboardingSectionAnalyticsProvider interface.
    /// It shall be created by the GettingStarted tab with the knowledge of the target package and the section id
    /// provided by the attribute of the onboarding section, so that the section implementer does not have to worry
    /// about it.
    /// </summary>
    internal class OnboardingSectionAnalyticsProvider : IOnboardingSectionAnalyticsProvider
    {
        readonly IMultiplayerCenterAnalytics m_Analytics;
        readonly string m_TargetPackageId;
        readonly string m_SectionId;

        public OnboardingSectionAnalyticsProvider(IMultiplayerCenterAnalytics analytics, string targetPackageId, string sectionId)
        {
            Debug.Assert(analytics != null);
            m_Analytics = analytics;
            m_TargetPackageId = targetPackageId;
            m_SectionId = sectionId;
        }
        
        public void SendInteractionEvent(InteractionDataType type, string displayName)
        {
            m_Analytics.SendGettingStartedInteractionEvent(m_TargetPackageId, m_SectionId, type, displayName);
        }
    }
}
