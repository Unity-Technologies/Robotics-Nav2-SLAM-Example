using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.Core;

public class ROSClockPublisher : MonoBehaviour
{
    public float PublishRateHz = 100f;

    float m_LastPublishTimeSeconds;

    ROSConnection m_ROS;

    float PublishPeriodSeconds => 1.0f / PublishRateHz;

    bool ShouldPublishMessage => Clock.TimeSeconds > m_LastPublishTimeSeconds + PublishPeriodSeconds;

    // Start is called before the first frame update
    void Start()
    {
        m_ROS = ROSConnection.instance;
        m_ROS.RegisterPublisher("clock", "rosgraph_msgs/Clock");
    }

    void PublishMessage()
    {
        var clockMsg = new TimeMsg
        {
            sec = (int)Clock.TimeSeconds,
            nanosec = (uint)((Clock.TimeSeconds - Math.Floor(Clock.TimeSeconds)) * 1000000000)
        };
        m_ROS.Send("clock", clockMsg);
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
