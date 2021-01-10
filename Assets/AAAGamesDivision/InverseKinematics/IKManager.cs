using System;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AAAGamesDivision
{
    namespace InverseKinematics
    {
        public class IKManager : MonoBehaviour
        {
            public IKJoint[] joints;
            public Transform follow;
            public bool followTarget = true;

            public float deltaStep = 0.1f;
            public float minLearningRate = 0.1f;
            public float maxLearningRate = 5f;
            public float minTargetDistance = 0.01f;
            public float maxTargetDistance = 0.1f;

            public int maxIterationsPerFrame = 20;
            private int iterationsPerFrame = 0;

            public float regularization = 0.8f;

            void Reset()
            {
                joints = GetComponentsInChildren(typeof(IKJoint))
                    .Select(x => (IKJoint)x)
                    .ToArray();
                follow = GameObject.Find("Controller").transform;
            }

            void Start()
            {
                StartCoroutine("IKSolver");
            }

            private Vector3 ForwardKinematics(IKJointAngles[] angles)
            {
                Quaternion rotation = transform.rotation;
                Vector3 position = joints[0].transform.position;
                for (int i = 0; i < angles.Length; ++i)
                {
                    rotation *= Quaternion.Euler(angles[i].Angles);
                    position += rotation * joints[i].arm;
                }
                return position;
            }

            private float DistanceToTarget(IKJointAngles[] angles)
            {
                return (follow.position - ForwardKinematics(angles)).magnitude;
            }

            private float LossFunction(IKJointAngles[] angles)
            {
                float distanceToTarget = DistanceToTarget(angles);
                float tension = angles.Select(x => x.Tension).Sum();
                float loss = distanceToTarget + regularization * tension;
                //Debug.Log($"loss = {loss:F3}; distance = {distanceToTarget:F3}; tension = {tension:F3}");
                return loss;
            }

            public Vector3[] LossGradient(IKJointAngles[] angles)
            {
                Vector3[] gradient = new Vector3[angles.Length];
                IKJointAngles[] deltaAngles = new IKJointAngles[angles.Length];
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

            public IEnumerator IKSolver()
            {
                IKJointAngles[] angles = joints.Select(x => x.Angles).ToArray();
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
                            angles[i].AdjustToConstraints();
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
    }
}
