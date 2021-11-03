using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace Unity.Robotics.Nav2SlamExample
{
    public static class TransformExtensions
    {

        public static TransformMsg ToRosTransform(this Transform tfUnity)
        {
            return new TransformMsg(
                // Using vector/quaternion To<>() because Transform.To<>() doesn't use localPosition/localRotation
                tfUnity.localPosition.To<FLU>(),
                tfUnity.localRotation.To<FLU>());
        }

        public static TransformStampedMsg ToRosTransformStamped(this Transform tfUnity, double timeStamp)
        {
            return new TransformStampedMsg(
                new HeaderMsg(new TimeStamp(timeStamp), tfUnity.parent.gameObject.name),
                tfUnity.gameObject.name,
                tfUnity.ToRosTransform());
        }
    }
}