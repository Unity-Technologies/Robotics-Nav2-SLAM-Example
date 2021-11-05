using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Rosgraph;

namespace Unity.Robotics.Nav2SlamExample
{
    /// <summary>
    /// Publishes the current time to the ROS /clock topic at the specified rate, using our <c>Clock</c> class as
    /// the time authority
    /// </summary>
    public class RosClockPublisher : MonoBehaviour
    {
        static readonly Clock.ClockMode s_ClockMode = Clock.ClockMode.UnityScaled;

        [SerializeField]
        double m_PublishRateHz = 100f;

        double m_LastPublishTimeSeconds;

        ROSConnection m_ROS;

        double PublishPeriodSeconds => 1.0f / m_PublishRateHz;

        bool ShouldPublishMessage => Clock.FrameStartTimeInSeconds - PublishPeriodSeconds > m_LastPublishTimeSeconds;

        void OnValidate()
        {
            var clocks = FindObjectsOfType<RosClockPublisher>();
            if (clocks.Length > 1)
            {
                Debug.LogWarning("Found too many clock publishers in the scene, there should only be one!");
            }
        }

        void Start()
        {
            Clock.Mode = s_ClockMode;
            m_ROS = ROSConnection.GetOrCreateInstance();
            m_ROS.RegisterPublisher<ClockMsg>("clock");
        }

        void PublishMessage()
        {
            var publishTime = Clock.time;
            var clockMsg = new TimeMsg
            {
                sec = (int)publishTime,
                nanosec = (uint)((publishTime - Math.Floor(publishTime)) * Clock.k_NanoSecondsInSeconds)
            };
            m_LastPublishTimeSeconds = publishTime;
            m_ROS.Publish("clock", clockMsg);
        }

        void Update()
        {
            if (ShouldPublishMessage)
            {
                PublishMessage();
            }
        }
    }
}