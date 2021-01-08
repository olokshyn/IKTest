using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RobotJoint : MonoBehaviour
{
    public Vector3 Axis;
    public Vector3 Arm;
    public float minAngle = -90;
    public float maxAngle = 90;
    public float restAngle = 0f;

    public static float GetTension(float angle, float restAngle, float minAngle, float maxAngle)
    {
        float closestLimit =
            Mathf.Abs(angle - minAngle) < Mathf.Abs(angle - maxAngle)
            ? minAngle
            : maxAngle;
        return
            Mathf.Abs(
                (angle - restAngle)
                / (restAngle - closestLimit)
            );
    }
    
    public float Angle
    {
        get => Vector3.Project(transform.localEulerAngles, Axis).magnitude;
        set => transform.localEulerAngles =
            Mathf.Clamp(value, minAngle, maxAngle) * Axis;
    }

    public float Tension => GetTension(Angle, restAngle, minAngle, maxAngle);

    void Reset()
    {
        RobotJoint[] joints = GetComponentsInChildren<RobotJoint>();
        if (joints.Length >= 2)
        {
            RobotJoint childJoint = joints[1];
            Arm = childJoint.transform.localPosition;
        }
        Axis = Vector3.right;
    }

    void Start()
    {
        Angle = restAngle;
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
