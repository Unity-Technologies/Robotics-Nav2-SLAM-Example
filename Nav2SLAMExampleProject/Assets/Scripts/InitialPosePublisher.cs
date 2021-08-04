using System;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using Unity.Robotics.Core;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class InitialPosePublisher : PoseRaycastPublisher
{
    // Start is called before the first frame update
    public override void Start()
    {
        base.Start();
        m_Ros.RegisterPublisher<PoseWithCovarianceStampedMsg>(m_Topic);
        m_DeselectedLabel = "Set Initial Pose";
        m_SelectedLabel = "Selecting Pose...";
    }

    // Override to ensure raycast is only valid for warehouse floor
    protected override (bool, RaycastHit) RaycastCheck(ClickState state)
    {
        var (didHit, hit) = base.RaycastCheck(state);
        didHit = didHit && hit.collider.gameObject.name.Contains("Floor");
        return (didHit, hit);
    }

    protected override bool ReleaseClick()
    {
        if (base.ReleaseClick())
        {
            m_Ros.Send(m_Topic, new PoseWithCovarianceStampedMsg
            {
                header = new HeaderMsg(new TimeStamp(Clock.time), "odom"),
                pose = new PoseWithCovarianceMsg
                {
                    pose = CalculatePoseMsg(),
                    covariance = new double[36]
                }
            });
            return true;
        }

        return false;
    }
}
