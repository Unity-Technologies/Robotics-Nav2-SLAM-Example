using System;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using UnityEngine.UI;

public class PointPublisher : RaycastPublisher
{
    // Start is called before the first frame update
    public override void Start()
    {
        base.Start();
        m_Ros.RegisterPublisher<PointStampedMsg>(m_Topic);
        m_Button = GameObject.Find("Canvas/Panel/PointButton").GetComponent<Button>();
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
            ClickPoint();
        }
    }

    public void BeginClickedPointButton()
    {
        m_Button.interactable = false;
        m_State = ClickState.BEGIN;
    }

    void ClickPoint()
    {
        var (didHit, hit) = RaycastCheck(ClickState.BEGIN);
        if (didHit)
        {
            m_Ros.Send(m_Topic, new PointStampedMsg
            {
                header = new HeaderMsg(new TimeStamp(Clock.time), "odom"),
                point = hit.point.To<FLU>()
            });
            m_Button.interactable = true;
            m_State = ClickState.NONE;
        }
    }
}
