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
    public float allowedError = 0.01f;

    public int maxIterationsPerFrame = 20;
    private int iterationsPerFrame = 0;

    public float regularization = 0.8f;

    void Reset()
    {
        joints = GetComponentsInChildren(typeof(RobotJoint))
            .Select(x => (RobotJoint)x)
            .ToArray();
    }

    void Start()
    {
        StartCoroutine("LossOptimization");
    }

    private Vector3 ForwardKinematics(Vector3[] angles)
    {
        Quaternion rotation = transform.rotation;
        Vector3 position = joints[0].transform.position;
        for (int i = 0; i < joints.Length; ++i)
        {
            rotation *= Quaternion.Euler(angles[i]);
            position += rotation * joints[i].arm;
        }
        return position;
    }

    private float LossFunction(Vector3[] angles)
    {
        Vector3 fk = ForwardKinematics(angles);
        float distanceToTarget = (follow.position - fk).magnitude;
        float tension =
            angles
            .Zip(
                joints,
                (angle, joint) => RobotJoint.GetTension(angle, joint)
            )
            .Sum()
        ;
        float loss = distanceToTarget + regularization * tension;
        Debug.Log($"loss = {loss:F3}; distance = {distanceToTarget:F3}; tension = {tension:F3}");
        return loss;
    }

    public Vector3[] LossGradient(Vector3[] angles)
    {
        Vector3[] gradient = new Vector3[angles.Length];
        Vector3[] deltaAngles = new Vector3[angles.Length];
        for (int i = 0; i < angles.Length; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                angles.CopyTo(deltaAngles, 0);
                deltaAngles[i][j] += deltaStep;

                float fx = LossFunction(angles);
                float fdx = LossFunction(deltaAngles);

                gradient[i][j] = (fdx - fx) / deltaStep;
            }
        }
        return gradient;
    }

    public IEnumerator LossOptimization()
    {
        Vector3[] angles = joints.Select(x => x.Angles).ToArray();
        iterationsPerFrame = 0;
        while (followTarget)
        {
            Vector3[] gradient = LossGradient(angles);
            float gradientMagnitude = Mathf.Sqrt(
                gradient
                    .Select(x => x.sqrMagnitude)
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
                .ToArray()
            ;
            for (int i = 0; i < angles.Length; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    angles[i][j] = Mathf.Clamp(
                        angles[i][j],
                        joints[i].minAngles[j],
                        joints[i].maxAngles[j]
                    );
                }
            }

            if (iterationsPerFrame == maxIterationsPerFrame)
            {
                iterationsPerFrame = 0;
                for (int i = 0; i < joints.Length; ++i)
                {
                    joints[i].Angles = angles[i];
                }
                yield return null;
            }
            else
            {
                ++iterationsPerFrame;
            }
        }
    }
}
