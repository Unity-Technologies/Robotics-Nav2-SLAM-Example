using System;
using System.Collections.Generic;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.Core;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine.Serialization;

public class LaserScanSensor : MonoBehaviour
{
    class ScanMarker
    {
        public GameObject SceneObject;
        public float TimeCurrentStateSeconds;

        internal Renderer Renderer => m_Renderer ??= SceneObject?.GetComponent<Renderer>();
        internal MaterialPropertyBlock PropertyBlock => m_PropertyBlock ??= new MaterialPropertyBlock();

        private Renderer m_Renderer;
        private MaterialPropertyBlock m_PropertyBlock;

        public void SetColor(Color color)
        {
            PropertyBlock.SetColor("_BaseColor", color);
            Renderer.SetPropertyBlock(PropertyBlock);
        }
    }

    public string topic;
    [FormerlySerializedAs("TimeBetweenScansSeconds")]
    public double PublishPeriodSeconds = 0.1;
    public float RangeMetersMin = 0;
    public float RangeMetersMax = 1000;
    public float ScanAngleStartDegrees = -45;
    public float ScanAngleEndDegrees = 45;
    // Change the scan start and end by this amount after every publish
    public float ScanOffsetAfterPublish = 0f;
    public int NumMeasurementsPerScan = 10;
    public float TimeBetweenMeasurementsSeconds = 0.01f;
    public GameObject markerPrefab;
    public string LayerMaskName = "TurtleBot3Manual";
    public string FrameId = "base_scan";
    
    [Header("Debug")]
    [SerializeField]
    bool m_RenderDebugVisuals;
    [SerializeField, FormerlySerializedAs("ActiveMarkerGradient")]
    Gradient m_ActiveMarkerGradient;
    [SerializeField, FormerlySerializedAs("InActiveMarkerGradient")]
    Gradient m_InActiveMarkerGradient;

    Queue<ScanMarker> m_MarkersActive = new Queue<ScanMarker>();
    Queue<ScanMarker> m_MarkersInactive = new Queue<ScanMarker>();

    float m_CurrentScanAngleStart;
    float m_CurrentScanAngleEnd;
    ROSConnection m_Ros;
    bool m_HaveWarnedNoMarkerPrefab;
    double m_TimeNextScanSeconds = -1;
    int m_NumMeasurementsTaken;
    List<float> ranges = new List<float>();
    LayerMask m_SelfMask;

    bool isScanning = false;
    double m_TimeLastScanBeganSeconds = -1;

    protected virtual void Start()
    {
        m_Ros = ROSConnection.instance;
        m_Ros.RegisterPublisher(topic, "sensor_msgs/LaserScan");

        m_CurrentScanAngleStart = ScanAngleStartDegrees;
        m_CurrentScanAngleEnd = ScanAngleEndDegrees;
        m_SelfMask = LayerMask.GetMask(LayerMaskName);

        m_TimeNextScanSeconds = Clock.Now + PublishPeriodSeconds;
    }

    void BeginScan()
    {
        isScanning = true;
        m_TimeLastScanBeganSeconds = Clock.Now;
        m_TimeNextScanSeconds = m_TimeLastScanBeganSeconds + PublishPeriodSeconds;
        m_NumMeasurementsTaken = 0;
        ResetMarkers();
    }

    void ResetMarkers()
    {
        var inactiveColor = m_InActiveMarkerGradient.Evaluate(0f);

        while (m_MarkersActive.Count > 0)
        {
            var marker = m_MarkersActive.Dequeue();
            //marker.SetActive(false);
            marker.SetColor(inactiveColor);
            marker.TimeCurrentStateSeconds = 0f;
            m_MarkersInactive.Enqueue(marker);
        }
    }

    public void EndScan()
    {
        if (ranges.Count == 0)
        {
            Debug.LogWarning($"Took {m_NumMeasurementsTaken} measurements but found no valid ranges");
        }
        else if (ranges.Count != m_NumMeasurementsTaken || ranges.Count != NumMeasurementsPerScan)
        {
            Debug.LogWarning($"Expected {NumMeasurementsPerScan} measurements. Actually took {m_NumMeasurementsTaken}" +
                             $"and recorded {ranges.Count} ranges.");
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
            ranges.Reverse();
        }

        var msg = new LaserScanMsg
        {
            header = new HeaderMsg
            {
                frame_id = FrameId,
                stamp = new TimeMsg
                {
                    sec = timestamp.Seconds,
                    nanosec = timestamp.NanoSeconds,
                }
            },
            range_min = RangeMetersMin,
            range_max = RangeMetersMax,
            angle_min = angleStartRos,
            angle_max = angleEndRos,
            angle_increment = (angleEndRos - angleStartRos) / NumMeasurementsPerScan,
            time_increment = TimeBetweenMeasurementsSeconds,
            scan_time = (float)PublishPeriodSeconds,
            intensities = new float[ranges.Count],
            ranges = ranges.ToArray(),
        };
        
        m_Ros.Send(topic, msg);

        m_NumMeasurementsTaken = 0;
        ranges.Clear();
        isScanning = false;
        var now = (float)Clock.time;
        if (now > m_TimeNextScanSeconds)
        {
            Debug.LogWarning($"Failed to complete scan started at {m_TimeLastScanBeganSeconds:F} before next scan was " +
                             $"scheduled to start: {m_TimeNextScanSeconds:F}, rescheduling to now ({now:F})");
            m_TimeNextScanSeconds = now;
        }

        m_CurrentScanAngleStart += ScanOffsetAfterPublish;
        m_CurrentScanAngleEnd += ScanOffsetAfterPublish;
        if (m_CurrentScanAngleStart > 360f || m_CurrentScanAngleEnd > 360f)
        {
            m_CurrentScanAngleStart -= 360f;
            m_CurrentScanAngleEnd -= 360f;
        }
    }

    void ActivateMarker(ScanMarker marker)
    {
        marker.SetColor(m_ActiveMarkerGradient.Evaluate(0f));
        marker.TimeCurrentStateSeconds = 0f;
        //marker.SetActive(true);
        m_MarkersActive.Enqueue(marker);
    }

    void UpdateAllMarkers()
    {
        if (!m_RenderDebugVisuals)
        {
            ResetMarkers();
            return;
        }
        var timeDelta = Clock.deltaTime;
        foreach (var marker in m_MarkersActive)
        {
            marker.TimeCurrentStateSeconds += timeDelta;
            var fadeAmount = Mathf.Clamp01(marker.TimeCurrentStateSeconds / (float)PublishPeriodSeconds);
            marker.SetColor(m_ActiveMarkerGradient.Evaluate(fadeAmount));
        }

        foreach (var marker in m_MarkersInactive)
        {
            marker.TimeCurrentStateSeconds += timeDelta;
            var fadeAmount = Mathf.Clamp01(marker.TimeCurrentStateSeconds / (float)PublishPeriodSeconds);
            marker.SetColor(m_InActiveMarkerGradient.Evaluate(fadeAmount));
        }
    }

    public void Update()
    {
        if (!isScanning)
        {
            if (Clock.NowTimeInSeconds < m_TimeNextScanSeconds)
            {
                return;
            }

            BeginScan();
        }


        var measurementsSoFar = TimeBetweenMeasurementsSeconds == 0 ? NumMeasurementsPerScan :
            1 + Mathf.FloorToInt((float)(Clock.time - m_TimeLastScanBeganSeconds) / TimeBetweenMeasurementsSeconds);
        if (measurementsSoFar > NumMeasurementsPerScan)
            measurementsSoFar = NumMeasurementsPerScan;

        var yawBaseDegrees = transform.rotation.eulerAngles.y;
        while (m_NumMeasurementsTaken < measurementsSoFar)
        {
            var t = m_NumMeasurementsTaken / (float)NumMeasurementsPerScan;
            var yawSensorDegrees = Mathf.Lerp(m_CurrentScanAngleStart, m_CurrentScanAngleEnd, t);
            var yawDegrees = yawBaseDegrees + yawSensorDegrees;
            var directionVector = Quaternion.Euler(0f, yawDegrees, 0f) * Vector3.forward;
            var measurementStart = RangeMetersMin * directionVector + transform.position;
            var measurementRay = new Ray(measurementStart, directionVector);
            var foundValidMeasurement = Physics.Raycast(measurementRay, out var hit, RangeMetersMax);
            // Only record measurement if it's within the sensor's operating range
            if (foundValidMeasurement)
            {
                ranges.Add(hit.distance);

                if (m_RenderDebugVisuals)
                {
                    if (m_MarkersInactive.Count > 0)
                    {
                        var marker = m_MarkersInactive.Dequeue();
                        ActivateMarker(marker);
                        marker.SceneObject.transform.position = hit.point;
                    }
                    else if (markerPrefab != null)
                    {
                        var scanMarker = new ScanMarker
                        {
                            SceneObject = Instantiate(markerPrefab, hit.point, Quaternion.identity),
                        };
                        ActivateMarker(scanMarker);
                    }
                    else if (!m_HaveWarnedNoMarkerPrefab)
                    {
                        Debug.LogWarning("No marker prefabs available to instantiate - won't be able to visualize scan");
                        m_HaveWarnedNoMarkerPrefab = true;
                    }
                }
            }
            else
            {
                ranges.Add(float.MaxValue);
            }

            // Even if Raycast didn't find a valid hit, we still count it as a measurement
            ++m_NumMeasurementsTaken;
        }
        
        UpdateAllMarkers();

        if (m_NumMeasurementsTaken >= NumMeasurementsPerScan)
        {
            if (m_NumMeasurementsTaken > NumMeasurementsPerScan)
            {
                Debug.LogError($"LaserScan has {m_NumMeasurementsTaken} measurements but we expected {NumMeasurementsPerScan}");
            }
            EndScan();
        }

    }
}
