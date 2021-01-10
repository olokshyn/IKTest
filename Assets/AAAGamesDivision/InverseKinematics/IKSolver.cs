using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace AAAGamesDivision
{
    namespace InverseKinematics
    {
        public class IKSolver
        {
            private IKJoint[] joints;
            private Transform ikTarget;
            private IKSolverParams ikParams;

            private bool isSolverRunning = false;

            public IKSolver(
                IKJoint[] joints,
                Transform ikTarget,
                IKSolverParams ikParams
            )
            {
                this.joints = joints;
                this.ikTarget = ikTarget;
                this.ikParams = ikParams;
            }

            private Vector3 ForwardKinematics(IKJointAngles[] angles)
            {
                Quaternion rotation = joints[0].transform.rotation;
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
                return (ikTarget.position - ForwardKinematics(angles)).magnitude;
            }

            private float LossFunction(IKJointAngles[] angles)
            {
                float distanceToTarget = DistanceToTarget(angles);
                float tension = angles.Select(x => x.Tension).Sum();
                float loss = distanceToTarget + ikParams.Regularization * tension;
                //Debug.Log($"loss = {loss:F3}; distance = {distanceToTarget:F3}; tension = {tension:F3}");
                return loss;
            }

            private Vector3[] LossGradient(IKJointAngles[] angles)
            {
                Vector3[] gradient = new Vector3[angles.Length];
                IKJointAngles[] deltaAngles = new IKJointAngles[angles.Length];
                for (int i = 0; i < angles.Length; ++i)
                {
                    for (int j = 0; j < 3; ++j)
                    {
                        angles.CopyTo(deltaAngles, 0);
                        deltaAngles[i][j] += ikParams.GradientDeltaStep;

                        float fx = LossFunction(angles);
                        float fdx = LossFunction(deltaAngles);

                        gradient[i][j] = (fdx - fx) / ikParams.GradientDeltaStep;
                    }
                }
                return gradient;
            }

            public IEnumerator StartSolver()
            {
                isSolverRunning = true;
                int iterationsPerFrame = 0;

                IKJointAngles[] angles = joints.Select(x => x.Angles).ToArray();

                while (isSolverRunning)
                {
                    float targetDistance = DistanceToTarget(angles);
                    if (targetDistance < ikParams.MinTargetDistance)
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
                        ikParams.MinLearningRate,
                        ikParams.MaxLearningRate,
                        (targetDistance - ikParams.MinTargetDistance)
                        / (ikParams.MaxTargetDistance - ikParams.MinTargetDistance)
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

                    if (iterationsPerFrame == ikParams.MaxIterationsPerFrame)
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

            public void StopSolver()
            {
                isSolverRunning = false;
            }
        }
    }
}
