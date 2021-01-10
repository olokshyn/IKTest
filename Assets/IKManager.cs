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
    public float minLearningRate = 0.1f;
    public float maxLearningRate = 5f;
    public float minTargetDistance = 0.03f;
    public float maxTargetDistance = 0.1f;

    public int maxIterationsPerFrame = 20;
    private int iterationsPerFrame = 0;

    public float regularization = 0.8f;

    void Reset()
    {
        joints = GetComponentsInChildren(typeof(RobotJoint))
            .Select(x => (RobotJoint)x)
            .ToArray();
        follow = GameObject.Find("Controller").transform;
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

    private float DistanceToTarget(Vector3[] angles)
    {
        return (follow.position - ForwardKinematics(angles)).magnitude;
    }

    private float LossFunction(Vector3[] angles)
    {
        float distanceToTarget = DistanceToTarget(angles);
        float tension =
            angles
            .Zip(
                joints,
                (angle, joint) => RobotJoint.GetTension(angle, joint)
            )
            .Sum()
        ;
        float loss = distanceToTarget + regularization * tension;
        //Debug.Log($"loss = {loss:F3}; distance = {distanceToTarget:F3}; tension = {tension:F3}");
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
            float targetDistance = DistanceToTarget(angles);
            if (targetDistance < minTargetDistance)
            {
                iterationsPerFrame = 0;
                Debug.Log($"Target reached, distance = {targetDistance:F3}");
                yield return null;
                continue;
            }

            Vector3[] gradient = LossGradient(angles);
            float gradientMagnitude = Mathf.Sqrt(
                gradient
                    .Select(x => x.sqrMagnitude)
                    .Sum()
            );

            float learningRate = Mathf.Lerp(
                minLearningRate,
                maxLearningRate,
                (targetDistance - minTargetDistance)
                / (maxTargetDistance - minTargetDistance)
            );
            Debug.Log($"learningRate = {learningRate:F3}; targetDistance = {targetDistance:F3}");

            for (int i = 0; i < angles.Length; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    angles[i][j] -= learningRate * gradient[i][j];
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
                continue;
            }
            else
            {
                ++iterationsPerFrame;
            }
        }
    }
}
