using System;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class Nav2PublisherTab : MonoBehaviour, IHudTab
{
    PointPublisher m_ClickedPointPublisher;
    GoalPosePublisher m_GoalPosePublisher;
    InitialPosePublisher m_InitialPosePublisher;

    public void Start()
    {
        HudPanel.RegisterTab(this);
        m_InitialPosePublisher = GetComponent<InitialPosePublisher>();
        m_ClickedPointPublisher = GetComponent<PointPublisher>();
        m_GoalPosePublisher = GetComponent<GoalPosePublisher>();
    }

    string IHudTab.Label => "Nav2 Publishers";

    void IHudTab.OnGUI(HudPanel hud)
    {
        GUILayout.BeginHorizontal();
        if (GUILayout.Button(m_InitialPosePublisher.CurrentLabel))
        {
            m_InitialPosePublisher.ToggleInteraction();
        }

        if (GUILayout.Button(m_ClickedPointPublisher.CurrentLabel))
        {
            m_ClickedPointPublisher.ToggleInteraction();
        }

        if (GUILayout.Button(m_GoalPosePublisher.CurrentLabel))
        {
            m_GoalPosePublisher.ToggleInteraction();
        }
        GUILayout.EndHorizontal();
    }

    void IHudTab.OnSelected() { }

    void IHudTab.OnDeselected() { }
}
