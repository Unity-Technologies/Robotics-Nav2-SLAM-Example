using System.Collections.Generic;
using RosMessageTypes.Geometry;
using Unity.Robotics.UrdfImporter;
using UnityEngine;

namespace Unity.Robotics.Nav2SlamExample
{
    class TransformTreeNode
    {
        readonly GameObject m_SceneObject;
        internal readonly List<TransformTreeNode> Children;
        public Transform Transform => m_SceneObject.transform;
        public string name => m_SceneObject.name;
        public bool IsALeafNode => Children.Count == 0;

        public TransformTreeNode(GameObject sceneObject)
        {
            m_SceneObject = sceneObject;
            Children = new List<TransformTreeNode>();
            PopulateChildNodes(this);
        }

        public static TransformStampedMsg ToTransformStamped(TransformTreeNode node)
        {
            return node.Transform.ToRosTransformStamped(Clock.time);
        }

        static void PopulateChildNodes(TransformTreeNode tfNode)
        {
            var parentTransform = tfNode.Transform;
            for (var childIndex = 0; childIndex < parentTransform.childCount; ++childIndex)
            {
                var childTransform = parentTransform.GetChild(childIndex);
                var childGO = childTransform.gameObject;

                // If game object has a URDFLink attached, it's a link in the transform tree
                if (childGO.TryGetComponent(out UrdfLink _))
                {
                    var childNode = new TransformTreeNode(childGO);
                    tfNode.Children.Add(childNode);
                }
            }
        }
    }
}