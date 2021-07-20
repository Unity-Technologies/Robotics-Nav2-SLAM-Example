using System.Collections.Generic;
using RosMessageTypes.Geometry;
using RosSharp.Urdf;
using UnityEngine;

internal class TransformTree
{
    public readonly GameObject SceneObject;
    public readonly List<TransformTree> Children;
    public Transform Transform => SceneObject.transform;
    public string parentName => SceneObject.transform.parent.name;
    public string name => SceneObject.name;
    public bool IsALeafNode => Children.Count == 0;

    public TransformTree(GameObject sceneObject)
    {
        SceneObject = sceneObject;
        Children = new List<TransformTree>();
        PopulateChildNodes(this);
    }

    public static TransformStampedMsg ToTransformStamped(TransformTree node)
    {
        return node.Transform.ToROSTransformStamped();
    }

    static void PopulateChildNodes(TransformTree tfNode)
    {
        var parentTransform = tfNode.Transform;
        for (var childIndex = 0; childIndex < parentTransform.childCount; ++childIndex)
        {
            var childTransform = parentTransform.GetChild(childIndex);
            var childGO = childTransform.gameObject;

            // If game object has a URDFLink attached, it's a link in the transform tree
            if (childGO.TryGetComponent(out UrdfLink _))
            {
                var childNode = new TransformTree(childGO);
                tfNode.Children.Add(childNode);
            }
        }
    }
}
