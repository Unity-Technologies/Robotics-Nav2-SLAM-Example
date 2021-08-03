using System;
using RosMessageTypes.Geometry;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class PosePublisher : RaycastPublisher
{
    BasicDrawing m_ArrowDrawing;
    Vector3[] m_MouseClicks = new Vector3[2];
    [SerializeField]
    Color m_Color;

    // Start is called before the first frame update
    public override void Start()
    {
        base.Start();
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
            BeginClick();
        }

        if (Input.GetMouseButton(0)) // update arrow
        {
            UpdateClick();
        }

        if (Input.GetMouseButtonUp(0)) // release click
        {
            ReleaseClick();
        }
    }

    void BeginClick()
    {
        var (didHit, hit) = RaycastCheck(ClickState.BEGIN);
        if (didHit)
        {
            m_MouseClicks[0] = hit.point;
            m_State = ClickState.UPDATE;
        }
    }

    void UpdateClick()
    {
        var (didHit, hit) = RaycastCheck(ClickState.UPDATE);
        if (didHit)
        {
            m_ArrowDrawing.Clear();
            m_ArrowDrawing.DrawArrow(m_MouseClicks[0], hit.point, m_Color);
        }
    }

    protected virtual bool ReleaseClick()
    {
        m_ArrowDrawing.Clear();
        var (didHit, hit) = RaycastCheck(ClickState.UPDATE);
        if (didHit)
        {
            m_MouseClicks[1] = hit.point;
            m_Button.interactable = true;
            m_State = ClickState.NONE;
        }

        return didHit;
    }

    protected PoseMsg CalculatePose()
    {
        var diff = (m_MouseClicks[1] - m_MouseClicks[0]).normalized;
        var rot = diff == Vector3.zero ? Quaternion.identity : Quaternion.LookRotation(diff);
        return new PoseMsg()
        {
            position = m_MouseClicks[0].To<FLU>(),
            orientation = rot.To<FLU>()
        };
    }
}