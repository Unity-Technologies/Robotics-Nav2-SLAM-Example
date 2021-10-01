using System.Collections;
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

namespace IntegrationTests 
{
    class WaypointTracker
    {
        internal Transform CurrentWaypoint => m_Waypoints[m_CurrentWaypointIdx];
        internal int Count => m_Waypoints.Count;
        
        const string k_WaypointTag = "Waypoint"; 
        List<Transform> m_Waypoints;
        int m_CurrentWaypointIdx;

        internal WaypointTracker()
        {
            var waypoints = GameObject.FindGameObjectsWithTag(k_WaypointTag).ToList();
            waypoints.Sort((g, o) => string.Compare(g.name, o.name));
            m_Waypoints = waypoints.Select(w => w.transform).ToList();
            m_CurrentWaypointIdx = -1;
            if (m_Waypoints.Count == 0)
            {
                Debug.LogWarning(
                    $"Found no GameObjects tagged with {k_WaypointTag} in {SceneManager.GetActiveScene().name}");
            }
        }

        internal bool NextWaypoint()
        {
            m_CurrentWaypointIdx++;
            return m_CurrentWaypointIdx < m_Waypoints.Count;
        }
    }

    [TestFixture, Explicit, Category("IntegrationTests")]
    public class NavigationIntegrationTests
    {
        // We don't use Path.Combine here because asset paths are always defined with forward-slash
        const string k_TestSceneAssetPath = "Scenes/Test/";
        static readonly List<string> k_TestSceneNames = new List<string>
        {
            "SimpleObstacleCourse"
        };

        // TODO: Use a "TestParameters" MonoBehaviour to hold these parameters in a GameObject per-scene,
        //       so they can be tuned without re-compiling code
        const string k_RobotTag = "robot";

        const string k_RobotBaseName = "base_footprint/base_link";
        const string k_GoalPoseFrameId = "map";
        const string k_GoalPoseTopic = "/goal_pose";
        
        const float k_Nav2InitializeTime = 5.0f;
        const float k_SleepBetweenWaypointsTime = 2.0f;
        
        // Used to define a timeout for waypoint navigation based on distances between steps
        const float k_MinimumSpeedExpected = 0.15f;

        // How close the TurtleBot must get to the navigation target to be successful
        const float k_DistanceSuccessThreshold = 0.4f;

        [UnityTearDown]
        public IEnumerator TearDown()
        {
            ROSConnection.GetOrCreateInstance().Disconnect();
            yield return null;
        }

        static string GetScenePath(string sceneName)
        {
            return k_TestSceneAssetPath + sceneName;
        }

        static bool IsCloseEnough(Transform expected, Transform actual)
        {
            return (expected.position - actual.position).magnitude < k_DistanceSuccessThreshold;
        }

        static void ToRosMsg(Transform transform, out RosMessageTypes.Geometry.PoseMsg poseMsg)
        {
            poseMsg = new RosMessageTypes.Geometry.PoseMsg();
            poseMsg.position = transform.position.To<FLU>();
            poseMsg.orientation = transform.rotation.To<FLU>();
        }

        static RosMessageTypes.Geometry.PoseStampedMsg ToRosMsg(Transform transform)
        {
            ToRosMsg(transform, out var pose);
            var msg = new RosMessageTypes.Geometry.PoseStampedMsg
            {
                header =
                {
                    stamp = new TimeStamp(Clock.time),
                    frame_id = k_GoalPoseFrameId
                },
                pose = pose
            };
            return msg;
        }

        [UnityTest]
        public IEnumerator TurtleBotOnObstacleCourse_NavigateWaypoints_Succeeds(
            [ValueSource(nameof(k_TestSceneNames))] string sceneName)
        {
            var scenePath = GetScenePath(sceneName);
            //SceneManager.LoadScene(scenePath);
            yield return SceneManager.LoadSceneAsync(scenePath);
            
            var ros = ROSConnection.GetOrCreateInstance();
            ros.ConnectOnStart = true;
            
            var robots = GameObject.FindGameObjectsWithTag(k_RobotTag);
            Assert.AreEqual(1, robots.Length,
                $"There should be exactly one object tagged '{k_RobotTag}' in the scene, but {scenePath} had {robots.Length}");
            var robot = robots[0].transform.Find(k_RobotBaseName)?.gameObject;
            Assert.IsNotNull(robot, 
                $"Expected {robots[0].name} to have a child object named {k_RobotBaseName} but it did not.");
            
            var waypoints = new WaypointTracker();
            Assert.Less(0, waypoints.Count, 
                $"Every test scene is expected to have at least one waypoint, but {scenePath} had none.");

            yield return new EnterPlayMode();
            // TODO: Implement some sort of confirmation mechanism on ROS side rather than use arbitrary sleep
            yield return new WaitForSeconds(k_Nav2InitializeTime);
            
            ros.RegisterPublisher<RosMessageTypes.Geometry.PoseStampedMsg>(k_GoalPoseTopic);

            while (waypoints.NextWaypoint())
            {
                var timeNavigationStarted = Time.realtimeSinceStartup;
                var waypoint = waypoints.CurrentWaypoint;
                var waypointTf = waypoint.transform;
                var robotTf = robot.transform;
                var distance = (waypointTf.position - robotTf.position).magnitude;
                var timeout = distance / k_MinimumSpeedExpected;
                
                
                ros.Send(k_GoalPoseTopic, ToRosMsg(waypointTf));

                yield return new WaitUntil(() => 
                    IsCloseEnough(waypointTf, robotTf) ||
                    Time.realtimeSinceStartup - timeNavigationStarted > timeout);
                
                Assert.IsTrue(IsCloseEnough(waypointTf.transform, robot.transform), 
                    $"Robot did not reach {waypoint.name} in the time allotted ({timeout} seconds).");
                
                // Because our success threshold may not match the navigation stack's success threshold, we "sleep" for
                // a small amount of time to ensure the nav stack has time to complete its route
                yield return new WaitForSeconds(k_SleepBetweenWaypointsTime);
            }
            
            Debug.Log($"Test completed successfully for {scenePath}");
            yield return null;
        }

    }

}