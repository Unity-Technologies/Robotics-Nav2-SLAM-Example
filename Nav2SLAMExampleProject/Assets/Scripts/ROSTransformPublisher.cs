using System.Collections.Generic;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.Tf2;

//using RosMessageTypes.Tf2;
using RosSharp.Urdf;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class ROSTransformPublisher : MonoBehaviour
{
    
    [SerializeField]
    float m_PublishRateHz = 20f;
    [SerializeField]
    List<string> m_GlobalFrameIds = new List<string> { "odom", "map" };
    
    float m_LastPublishTimeSeconds;

    TransformTree m_TransformRoot;
    ROSConnection m_ROS;

    float PublishPeriodSeconds => 1.0f / m_PublishRateHz;

    bool ShouldPublishMessage => Clock.TimeSeconds > m_LastPublishTimeSeconds + PublishPeriodSeconds;

    // Start is called before the first frame update
    void Start()
    {
        m_ROS = ROSConnection.instance;
        m_TransformRoot = new TransformTree(gameObject);
        m_ROS.RegisterPublisher("tf", "tf2_msgs/TFMessage");

    }

    void PopulateTFList(List<TransformStampedMsg> tfList, TransformTree tfNode)
    {
        var parentFrame = tfNode.name;
        // TODO: Some of this could be done once and cached rather than doing from scratch every time
        foreach (var childTf in tfNode.Children)
        {
            tfList.Add(TransformTree.ToTransformStamped(childTf));

            if (!childTf.IsALeafNode)
            {
                PopulateTFList(tfList, childTf);
            }
        }
    }

    void PublishMessage()
    {
        var tfMessageList = new List<TransformStampedMsg>();
        foreach (var frameId in m_GlobalFrameIds)
        {
            var tfRootToGlobal = new TransformStampedMsg(
                new HeaderMsg(Clock.TimeStamp, frameId),
                m_TransformRoot.name,
                m_TransformRoot.Transform.To<FLU>());
            tfMessageList.Add(tfRootToGlobal);
        }

        PopulateTFList(tfMessageList, m_TransformRoot);

        var tfMessage = new TFMessageMsg(tfMessageList.ToArray());
        m_ROS.Send("tf", tfMessage);
        m_LastPublishTimeSeconds = Clock.TimeSeconds;
    }

    void Update()
    {
        if (ShouldPublishMessage)
        {
            PublishMessage();
        }

    }
}
