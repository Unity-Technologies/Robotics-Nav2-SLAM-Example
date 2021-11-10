using System;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.UrdfImporter.Control;
using UnityEngine;
using UnityEngine.Serialization;

namespace Unity.Robotics.Nav2SlamExample
{
    /// <summary>
    /// The Autonomous Ground Vehicle Controller is an adapter layer between either ROS or the Unity Input system and
    /// the <c>ArticulationBody</c> class's joint controllers. Given the specifications of a specific vehicle,
    /// this class will read commanded linear and angular velocity commands and translate them to the appropriate
    /// control commands for assigned wheel joints.
    /// </summary>
    public class AgvController : MonoBehaviour
    {
        enum ControlMode
        {
            Keyboard,
            ROS
        }

        [SerializeField, FormerlySerializedAs("wheel1")]
        GameObject m_WheelLeft;
        [SerializeField, FormerlySerializedAs("wheel2")]
        GameObject m_WheelRight;
        [SerializeField]
        ControlMode m_Mode = ControlMode.ROS;

        [SerializeField]
        [Tooltip("Max linear speed the vehicle will be allowed to reach, in meters per second.")]
        float m_MaxLinearSpeed = 2; //  m/s
        [SerializeField]
        [Tooltip("Max rate the vehicle will be allowed to rotate around its y-axis, in radians per second")]
        float m_MaxRotationalSpeed = 1; //
        [SerializeField]
        [Tooltip("Radius of each wheel (assumed to be the same) in meters.")]
        float m_WheelRadius = 0.033f; //meters
        [SerializeField]
        [Tooltip("Distance between the inner face of the wheels, in meters")]
        float m_TrackWidth = 0.288f;
        [SerializeField]
        [Tooltip("Force limit be to set on the assigned ArticulationBody link - fine to leave as default.")]
        float m_ForceLimit = 10;
        [SerializeField]
        [Tooltip("Damping to be set on the assigned ArticulationBody link - fine to leave as default.")]
        float m_Damping = 10;

        [SerializeField]
        [Tooltip("Time in seconds since the last velocity command before the vehicle decides to stop moving.")]
        float m_RosTimeout = 0.5f;

        ArticulationBody m_LeftJoint;
        ArticulationBody m_RightJoint;

        float m_TimeLastCommandReceived;
        ROSConnection m_ROS;
        float m_VelocityLinear;
        float m_VelocityAngular;

        void Start()
        {
            m_LeftJoint = m_WheelLeft.GetComponent<ArticulationBody>();
            m_RightJoint = m_WheelRight.GetComponent<ArticulationBody>();
            InitializeJoint(m_LeftJoint);
            InitializeJoint(m_RightJoint);
            m_ROS = ROSConnection.GetOrCreateInstance();
            m_ROS.Subscribe<TwistMsg>("cmd_vel", ProcessRosCommand);
        }

        void ProcessRosCommand(TwistMsg cmdVel)
        {
            // Convert from ROS body coordinates (Front, Left, Up) to Unity coordinates (Right, Up, Forward)
            var linear = cmdVel.linear.From<FLU>();
            // z is forward in Unity coordinates
            m_VelocityLinear = linear.z;
            // This is a roll, pitch, yaw vector in right-handed coordinates, i.e. NED (North, East, Down)
            var angular = cmdVel.angular.From<NED>();
            // y is up in Unity coordinates -- we're reading the yaw command
            m_VelocityAngular = angular.y;
            m_TimeLastCommandReceived = Time.time;
        }

        void ProcessKeyboardInput()
        {
            var moveDirection = Input.GetAxis("Vertical");
            m_VelocityLinear = Mathf.Approximately(0, moveDirection) ? 0
                : Mathf.Sign(moveDirection) * m_MaxLinearSpeed;

            var turnDirection = Input.GetAxis("Horizontal");
            m_VelocityAngular = Mathf.Approximately(0, turnDirection) ? 0
                : Mathf.Sign(turnDirection) * m_MaxLinearSpeed;
        }

        void Update()
        {
            switch (m_Mode)
            {
                case ControlMode.Keyboard:
                    ProcessKeyboardInput();
                    break;
                case ControlMode.ROS:
                    if (Time.time - m_TimeLastCommandReceived > m_RosTimeout)
                    {
                        m_VelocityLinear = 0f;
                        m_VelocityAngular = 0f;
                    }
                    break;
                default:
                    throw new NotImplementedException(
                        $"No handling defined for {nameof(ControlMode)}.{m_Mode}");
            }
        }

        void FixedUpdate()
        {
            m_VelocityLinear = Mathf.Clamp(m_VelocityLinear, -m_MaxLinearSpeed, m_MaxLinearSpeed);
            m_VelocityAngular = Mathf.Clamp(m_VelocityAngular, -m_MaxRotationalSpeed, m_MaxRotationalSpeed);

            SetWheelRotationalVelocity(m_LeftJoint, RotationDirection.Positive);
            SetWheelRotationalVelocity(m_RightJoint, RotationDirection.Negative);
        }

        void InitializeJoint(ArticulationBody joint)
        {
            var drive = joint.xDrive;
            drive.forceLimit = m_ForceLimit;
            drive.damping = m_Damping;
            joint.xDrive = drive;
        }

        void SetWheelRotationalVelocity(ArticulationBody joint, RotationDirection wheelRotationDirection)
        {
            var wheelVelocity = (m_VelocityLinear / m_WheelRadius) * Mathf.Rad2Deg;
            if (m_VelocityAngular != 0)
            {
                var directionMultiplier =
                    wheelRotationDirection switch
                    {
                        RotationDirection.Positive => 1,
                        RotationDirection.Negative => -1,
                        _ => throw new NotSupportedException(
                            $"{nameof(RotationDirection)} cannot be set to {wheelRotationDirection}"),
                    };
                var wheelSpeedDiff = m_VelocityAngular * (m_TrackWidth / m_WheelRadius) * directionMultiplier;
                wheelVelocity += wheelSpeedDiff * Mathf.Rad2Deg;
            }
            var drive = joint.xDrive;
            drive.targetVelocity = wheelVelocity;
            joint.xDrive = drive;
        }
    }
}
