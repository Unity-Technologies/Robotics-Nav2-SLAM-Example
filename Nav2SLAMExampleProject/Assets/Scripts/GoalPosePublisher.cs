using System;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using Unity.Robotics.Core;
using Unity.Robotics.MessageVisualizers;
using UnityEngine;

public class GoalPosePublisher : PoseRaycastPublisher
{
    // Start is called before the first frame update
    public override void Start()
    {
        base.Start();
        m_Ros.RegisterPublisher<PoseStampedMsg>(m_Topic);
        m_DeselectedLabel = "Set Goal";
        m_SelectedLabel = "Selecting Goal...";
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
            m_Ros.Send(m_Topic, new PoseStampedMsg()
            {
                header = new HeaderMsg(new TimeStamp(Clock.time), "odom"),
                pose = CalculatePoseMsg()
            });
            return true;
        }
    
        return false;
    }
}
