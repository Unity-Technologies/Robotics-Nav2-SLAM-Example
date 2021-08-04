using System;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using Unity.Robotics.Core;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class PointPublisher : RaycastPublisher
{
    // Start is called before the first frame update
    public override void Start()
    {
        base.Start();
        m_Ros.RegisterPublisher<PointStampedMsg>(m_Topic);
        m_DeselectedLabel = "Publish Point";
        m_SelectedLabel = "Selecting Point...";
    }

    // Update is called once per frame
    void Update()
    {
        if (!ValidClick())
        {
            return;
        }

        if (Input.GetMouseButtonDown(0))
        {
            ClickPoint();
        }
    }

    // Override to ensure raycast is only valid for warehouse floor
    protected override (bool, RaycastHit) RaycastCheck(ClickState state)
    {
        var (didHit, hit) = base.RaycastCheck(state);
        didHit = didHit && hit.collider.gameObject.name.Contains("Floor");
        return (didHit, hit);
    }

    void ClickPoint()
    {
        var (didHit, hit) = RaycastCheck(ClickState.Started);
        if (didHit)
        {
            m_Ros.Send(m_Topic, new PointStampedMsg
            {
                header = new HeaderMsg(new TimeStamp(Clock.time), "odom"),
                point = hit.point.To<FLU>()
            });
            m_State = ClickState.None;
        }
    }
}
