using System;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using Unity.Robotics.Core;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using UnityEngine.UI;

public class GoalPosePublisher : RaycastPublisher
{
    BasicDrawing m_ArrowDrawing;
    Vector3[] m_MouseClicks = new Vector3[2];

    // Start is called before the first frame update
    public override void Start()
    {
        base.Start();
        m_Ros.RegisterPublisher<PoseStampedMsg>(m_Topic);
        m_Button = GameObject.Find("Canvas/Panel/GoalButton").GetComponent<Button>();
        m_ArrowDrawing = BasicDrawingManager.CreateDrawing();
    }

    // Update is called once per frame
    void Update()
    {
        if (!ValidClick())
        {
            return;
        }

        if (Input.GetMouseButtonDown(0)) // begin click
        {
            GoalPoseClick();
        }

        if (Input.GetMouseButton(0)) // update arrow
        {
            GoalPoseUpdate();
        }

        if (Input.GetMouseButtonUp(0)) // release click
        {
            GoalPoseRelease();
        }
    }

    public void BeginGoalPoseButton()
    {
        m_Button.interactable = false;
        m_State = ClickState.BEGIN;
    }

    void GoalPoseClick()
    {
        var (didHit, hit) = RaycastCheck(ClickState.BEGIN);
        if (didHit)
        {
            m_MouseClicks[0] = hit.point;
            m_State = ClickState.UPDATE;
        }
    }

    void GoalPoseUpdate()
    {
        var (didHit, hit) = RaycastCheck(ClickState.UPDATE);
        if (didHit)
        {
            m_ArrowDrawing.Clear();
            m_ArrowDrawing.DrawArrow(m_MouseClicks[0], hit.point, Color.green);
        }
    }

    void GoalPoseRelease()
    {
        m_ArrowDrawing.Clear();
        var (didHit, hit) = RaycastCheck(ClickState.UPDATE);
        if (didHit)
        {
            m_MouseClicks[1] = hit.point;
            var diff = (m_MouseClicks[1] - m_MouseClicks[0]).normalized;
            var rot = diff == Vector3.zero ? Quaternion.identity : Quaternion.LookRotation(diff);

            var goalMessage = new PoseStampedMsg
            {
                header = new HeaderMsg(new TimeStamp(Clock.time), "odom"),
                pose = new PoseMsg
                {
                    position = m_MouseClicks[0].To<FLU>(),
                    orientation = rot.To<FLU>()
                }
            };
            m_Ros.Send(m_Topic, goalMessage);
            m_Button.interactable = true;
            m_State = ClickState.NONE;
        }
    }
}
