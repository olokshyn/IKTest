using System;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class IKManager : MonoBehaviour
{
    public RobotJoint[] joints;
    public Transform follow;
    public bool followTarget = true;

    public float deltaStep = 0.1f;
    public float learningRate = 5f;
    public float allowedError = 0.001f;

    public int maxIterationsPerFrame = 20;
    private int iterationsPerFrame = 0;

    public float regularization = 1f;

    // Update is called once per frame
    void Update()
    {
        
    }

    void Reset()
    {
        joints = GetComponentsInChildren(typeof(RobotJoint))
            .Select(x => (RobotJoint)x)
            .ToArray()
        ;
    }

    void Start()
    {
        StartCoroutine("LossOptimization");
    }

    private Vector3 ForwardKinematics(float[] angles)
    {
        Quaternion rotation = transform.rotation;
        Vector3 position = joints[0].transform.position;
        for (int i = 0; i < joints.Length; ++i)
        {
            rotation *= Quaternion.AngleAxis(angles[i], joints[i].Axis);
            position += rotation * joints[i].Arm;
        }
        return position;
    }

    private float LossFunction(float[] angles)
    {
        Vector3 fk = ForwardKinematics(angles);
        float distanceToTarget = (follow.position - fk).magnitude;
        float tension =
            angles
            .Zip(
                joints,
                (angle, joint) => RobotJoint.GetTension(
                    angle, joint.restAngle, joint.minAngle, joint.maxAngle
                )
            )
            .Sum()
        ;
        float loss = distanceToTarget + regularization * tension;
        Debug.Log($"loss = {loss:F3}; distance = {distanceToTarget:F3}; tension = {tension:F3}");
        return loss;
    }

    public float[] LossGradient(float[] angles)
    {
        float[] gradient = new float[angles.Length];
        float[] deltaAngles = new float[angles.Length];
        for (int i = 0; i < angles.Length; ++i)
        {
            angles.CopyTo(deltaAngles, 0);
            deltaAngles[i] += deltaStep;

            float fx = LossFunction(angles);
            float fdx = LossFunction(deltaAngles);

            gradient[i] = (fdx - fx) / deltaStep;
        }
        return gradient;
    }

    public IEnumerator LossOptimization()
    {
        float[] angles = joints.Select(x => x.Angle).ToArray();
        iterationsPerFrame = 0;
        while (followTarget)
        {
            float[] gradient = LossGradient(angles);
            float gradientMagnitude = Mathf.Sqrt(
                gradient
                    .Select(x => Mathf.Pow(x, 2))
                    .Sum()
            );
            if (gradientMagnitude < allowedError)
            {
                iterationsPerFrame = 0;
                yield return null;
            }

            angles =
                angles
                .Zip(
                    gradient,
                    (angle, gradient) => angle - learningRate * gradient
                )
                .Zip(
                    joints,
                    (angle, joint) =>
                        Mathf.Clamp(angle, joint.minAngle, joint.maxAngle)
                )
                .ToArray()
            ;

            for (int i = 0; i < joints.Length; ++i)
            {
                joints[i].Angle = angles[i];
            }

            if (iterationsPerFrame == maxIterationsPerFrame)
            {
                iterationsPerFrame = 0;
                yield return null;
            }
            else
            {
                ++iterationsPerFrame;
            }
        }
    }
}
