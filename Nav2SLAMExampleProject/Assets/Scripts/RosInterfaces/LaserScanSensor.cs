using System;
using System.Collections.Generic;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine.Serialization;

namespace Unity.Robotics.Nav2SlamExample
{
    public class LaserScanSensor : MonoBehaviour
    {
        [SerializeField]
        string m_Topic = "/scan";
        [SerializeField]
        double m_PublishPeriodSeconds = 0.1;
        [SerializeField, FormerlySerializedAs("RangeMetersMin")]
        float m_RangeMetersMin;
        [SerializeField, FormerlySerializedAs("RangeMetersMax")]
        float m_RangeMetersMax = 1000;
        [SerializeField, FormerlySerializedAs("ScanAngleStartDegrees")]
        float m_ScanAngleStartDegrees = -45;
        [SerializeField, FormerlySerializedAs("ScanAngleEndDegrees")]
        float m_ScanAngleEndDegrees = 45;
        // Change the scan start and end by this amount after every publish
        [SerializeField]
        float m_ScanOffsetAfterPublish;
        [SerializeField, FormerlySerializedAs("NumMeasurementsPerScan")]
        int m_NumMeasurementsPerScan = 10;
        [SerializeField]
        string m_FrameId = "base_scan";

        float m_CurrentScanAngleStart;
        float m_CurrentScanAngleEnd;
        ROSConnection m_Ros;
        double m_TimeNextScanSeconds = -1;
        int m_NumMeasurementsTaken;
        List<float> m_Measurements = new List<float>();

        bool m_IsScanning;
        double m_TimeLastScanBeganSeconds = -1;

        void Start()
        {
            m_Ros = ROSConnection.GetOrCreateInstance();
            m_Ros.RegisterPublisher<LaserScanMsg>(m_Topic);

            m_CurrentScanAngleStart = m_ScanAngleStartDegrees;
            m_CurrentScanAngleEnd = m_ScanAngleEndDegrees;

            m_TimeNextScanSeconds = Clock.Now + m_PublishPeriodSeconds;
        }

        void BeginScan()
        {
            m_IsScanning = true;
            m_TimeLastScanBeganSeconds = Clock.Now;
            m_TimeNextScanSeconds = m_TimeLastScanBeganSeconds + m_PublishPeriodSeconds;
            m_NumMeasurementsTaken = 0;
        }

        void EndScan()
        {
            if (m_Measurements.Count == 0)
            {
                Debug.LogWarning($"Took {m_NumMeasurementsTaken} measurements but found no valid ranges");
            }
            else if (m_Measurements.Count != m_NumMeasurementsTaken || m_Measurements.Count != m_NumMeasurementsPerScan)
            {
                Debug.LogWarning($"Expected {m_NumMeasurementsPerScan} measurements. Actually took {m_NumMeasurementsTaken}" +
                    $"and recorded {m_Measurements.Count} ranges.");
            }

            var timestamp = new TimeStamp(Clock.time);
            // Invert the angle ranges when going from Unity to ROS
            var angleStartRos = -m_CurrentScanAngleStart * Mathf.Deg2Rad;
            var angleEndRos = -m_CurrentScanAngleEnd * Mathf.Deg2Rad;
            if (angleStartRos > angleEndRos)
            {
                Debug.LogWarning("LaserScan was performed in a clockwise direction but ROS expects a counter-clockwise scan, flipping the ranges...");
                var temp = angleEndRos;
                angleEndRos = angleStartRos;
                angleStartRos = temp;
                m_Measurements.Reverse();
            }

            var msg = new LaserScanMsg
            {
                header = new HeaderMsg
                {
                    frame_id = m_FrameId,
                    stamp = new TimeMsg
                    {
                        sec = timestamp.Seconds,
                        nanosec = timestamp.NanoSeconds,
                    }
                },
                range_min = m_RangeMetersMin,
                range_max = m_RangeMetersMax,
                angle_min = angleStartRos,
                angle_max = angleEndRos,
                angle_increment = (angleEndRos - angleStartRos) / m_NumMeasurementsPerScan,
                // This is an ideal LIDAR, so it takes all its measurements instantaneously
                time_increment = 0,
                scan_time = (float)m_PublishPeriodSeconds,
                intensities = new float[m_Measurements.Count],
                ranges = m_Measurements.ToArray(),
            };

            m_Ros.Publish(m_Topic, msg);

            m_NumMeasurementsTaken = 0;
            m_Measurements.Clear();
            m_IsScanning = false;
            var now = (float)Clock.time;
            if (now > m_TimeNextScanSeconds)
            {
                Debug.LogWarning($"Failed to complete scan started at {m_TimeLastScanBeganSeconds:F} before next scan was " +
                    $"scheduled to start: {m_TimeNextScanSeconds:F}, rescheduling to now ({now:F})");
                m_TimeNextScanSeconds = now;
            }

            m_CurrentScanAngleStart += m_ScanOffsetAfterPublish;
            m_CurrentScanAngleEnd += m_ScanOffsetAfterPublish;
            if (m_CurrentScanAngleStart > 360f || m_CurrentScanAngleEnd > 360f)
            {
                m_CurrentScanAngleStart -= 360f;
                m_CurrentScanAngleEnd -= 360f;
            }
        }

        void Update()
        {
            if (!m_IsScanning)
            {
                if (Clock.NowTimeInSeconds < m_TimeNextScanSeconds)
                {
                    return;
                }

                BeginScan();
            }


            // TODO: This could be optimized by using RaycastCommand
            var yawBaseDegrees = transform.rotation.eulerAngles.y;
            while (m_NumMeasurementsTaken < m_NumMeasurementsPerScan)
            {
                var t = m_NumMeasurementsTaken / (float)m_NumMeasurementsPerScan;
                var yawSensorDegrees = Mathf.Lerp(m_CurrentScanAngleStart, m_CurrentScanAngleEnd, t);
                var yawDegrees = yawBaseDegrees + yawSensorDegrees;
                var directionVector = Quaternion.Euler(0f, yawDegrees, 0f) * Vector3.forward;
                var measurementStart = m_RangeMetersMin * directionVector + transform.position;
                var measurementRay = new Ray(measurementStart, directionVector);
                var foundValidMeasurement = Physics.Raycast(measurementRay, out var hit, m_RangeMetersMax);
                // Measurements outside of the sensor's max distance will be reported as MaxValue. In some cases,
                // they may need to be filtered on the receiving end
                m_Measurements.Add(foundValidMeasurement ? hit.distance : float.MaxValue);

                // Even if Raycast didn't find a valid hit, we still count it as a measurement
                ++m_NumMeasurementsTaken;
            }

            EndScan();
        }
    }
}
