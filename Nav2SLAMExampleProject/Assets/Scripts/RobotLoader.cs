using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.Control;

public class RobotLoader : MonoBehaviour
{
    void Start()
    {
        //Tile Constraint
        GameObject tiltConstraint = new GameObject("TiltConstraint");
        tiltConstraint.transform.position = transform.position;
        ConfigurableJoint constraint = tiltConstraint.AddComponent<ConfigurableJoint>();
        constraint.GetComponent<Rigidbody>().isKinematic = true;
        constraint.angularXMotion = ConfigurableJointMotion.Locked;
        constraint.angularZMotion = ConfigurableJointMotion.Locked;
        GameObject baseLink = transform.Find("base_footprint/base_link").gameObject;
        constraint.connectedArticulationBody = baseLink.GetComponent<ArticulationBody>();

        //Remove Collisions
        Collider[] casterCollider = baseLink.transform.Find("caster_back_right_link").GetComponentsInChildren<Collider>();
        foreach (Collider col in casterCollider)
        {
            Destroy(col);
        }

        Collider[] casterCollider2 = baseLink.transform.Find("caster_back_left_link").GetComponentsInChildren<Collider>();
        foreach (Collider col in casterCollider2)
        {
            Destroy(col);
        }

        //TensorUpdate
        Vector3 inertiaup = new Vector3(1, 1, 1);
        GameObject wheel1 = baseLink.transform.Find("wheel_left_link").gameObject;
        GameObject wheel2 = baseLink.transform.Find("wheel_right_link").gameObject;
        InertiaTensorUpdate wheel1up = wheel1.gameObject.AddComponent<InertiaTensorUpdate>();
        InertiaTensorUpdate wheel2up = wheel2.gameObject.AddComponent<InertiaTensorUpdate>();
        wheel1up.inertiaTensor = wheel2up.inertiaTensor = inertiaup;
    }
}
