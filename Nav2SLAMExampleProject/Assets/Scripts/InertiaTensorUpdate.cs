using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class InertiaTensorUpdate : MonoBehaviour
{
    public Vector3 inertiaTensor;

    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        GetComponent<ArticulationBody>().inertiaTensor = inertiaTensor;
    }
}