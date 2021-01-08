using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RobotJoint : MonoBehaviour
{
    public Vector3 arm;
    public Vector3 restAngles = new Vector3(0f, 0f, 0f);
    public Vector3 minAngles = new Vector3(-90f, -90f, -90f);
    public Vector3 maxAngles = new Vector3(90f, 90f, 90f);

    public static float GetTension(Vector3 angles, RobotJoint joint)
    {
        float tension = 0;
        for (int i = 0; i < 3; ++i)
        {
            float closestLimit =
                Mathf.Abs(angles[i] - joint.minAngles[i]) < Mathf.Abs(angles[i] - joint.maxAngles[i])
                ? joint.minAngles[i]
                : joint.maxAngles[i];
            tension +=
                Mathf.Abs(
                    (angles[i] - joint.restAngles[i])
                    / (joint.restAngles[i] - closestLimit)
                );
        }
        return tension / 3f;
    }
    
    public Vector3 Angles
    {
        get => transform.localEulerAngles;
        set
        {
            for (int i = 0; i < 3; ++i)
            {
                value[i] = Mathf.Clamp(value[i], minAngles[i], maxAngles[i]);
            }
            transform.localEulerAngles = value;
        }
    }

    public float Tension => GetTension(Angles, this);

    void Reset()
    {
        RobotJoint[] joints = GetComponentsInChildren<RobotJoint>();
        if (joints.Length >= 2)
        {
            RobotJoint childJoint = joints[1];
            arm = childJoint.transform.localPosition;
        }
    }

    void Start()
    {
        Angles = restAngles;
    }
}
