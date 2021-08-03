using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using Unity.Robotics.Core;
using UnityEngine;
using UnityEngine.UI;

public class InitialPosePublisher : PosePublisher
{
    // Start is called before the first frame update
    public override void Start()
    {
        base.Start();
        m_Ros.RegisterPublisher<PoseWithCovarianceStampedMsg>(m_Topic);
        m_Button = GameObject.Find("Canvas/Panel/InitialButton").GetComponent<Button>();
    }

    protected override bool ReleaseClick()
    {
        if (base.ReleaseClick())
        {
            m_Ros.Send(m_Topic, new PoseWithCovarianceStampedMsg()
            {
                header = new HeaderMsg(new TimeStamp(Clock.time), "odom"),
                pose = new PoseWithCovarianceMsg()
                {
                    pose = CalculatePose(),
                    covariance = new double[36]
                }
            });
            return true;
        }
        return false;
    }
}
