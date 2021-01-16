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
            private IKEffectorInfo[] effectors;
            private IKSolverParams ikParams;
            private bool isSolverRunning = false;

            public IKSolver(IKEffectorInfo[] effectors, IKSolverParams ikParams)
            {
                this.effectors = effectors;
                this.ikParams = ikParams;
            }

            private float DistanceToTarget(Dictionary<string, IKJointAngles> angles)
            {
                return
                    effectors
                    .Select(effector => effector.DistanceToTarget(angles))
                    .Sum();
            }

            private float LossFunction(Dictionary<string, IKJointAngles> angles)
            {
                float distanceToTarget = DistanceToTarget(angles);
                float tension =
                    angles
                    .Values
                    .Select(angle => angle.Tension)
                    .Sum();
                float loss = distanceToTarget + ikParams.Regularization * tension;
                //Debug.Log($"loss = {loss:F3}; distance = {distanceToTarget:F3}; tension = {tension:F3}");
                return loss;
            }

            private Dictionary<string, Vector3>
                LossGradient(Dictionary<string, IKJointAngles> angles)
            {
                Dictionary<string, IKJointAngles> perturbedAngles =
                    angles
                    .ToDictionary(entry => entry.Key, entry => entry.Value);
                Dictionary<string, Vector3> gradient =
                    angles
                    .ToDictionary(entry => entry.Key, entry => Vector3.zero);

                foreach (var jointAngle in angles)
                {
                    IKJointAngles oldAngles = perturbedAngles[jointAngle.Key];
                    Vector3 localGradient = Vector3.zero;
                    for (int i = 0; i < 3; ++i)
                    {
                        IKJointAngles newAngles = oldAngles;
                        newAngles[i] += ikParams.GradientDeltaStep;
                        perturbedAngles[jointAngle.Key] = newAngles;

                        float funcValue = LossFunction(angles);
                        float funcValuePerturbed = LossFunction(perturbedAngles);
                        localGradient[i] = (funcValuePerturbed - funcValue) / ikParams.GradientDeltaStep;

                        perturbedAngles[jointAngle.Key] = oldAngles;
                    }
                    gradient[jointAngle.Key] = localGradient;
                }
                return gradient;
            }

            public IEnumerator StartSolver()
            {
                isSolverRunning = true;
                int iterationsPerFrame = 0;

                Dictionary<string, IKJointAngles> angles =
                    effectors
                    .SelectMany(effector => effector.Joints)
                    .GroupBy(joint => joint.name)
                    .ToDictionary(
                        group => group.Key,
                        group => {
                            var angles = group.First().Angles;
                            angles.ToRest();
                            return angles;
                        }
                    );

                while (isSolverRunning)
                {
                    float targetDistance = DistanceToTarget(angles);
                    if (targetDistance < ikParams.MinTargetDistance)
                    {
                        iterationsPerFrame = 0;
                        //Debug.Log($"Target reached, distance = {targetDistance:F3}");
                        yield return null;
                        continue;
                    }

                    float learningRate = Mathf.Lerp(
                        ikParams.MinLearningRate,
                        ikParams.MaxLearningRate,
                        (targetDistance - ikParams.MinTargetDistance)
                        / (ikParams.MaxTargetDistance - ikParams.MinTargetDistance)
                    );
                    //Debug.Log($"learningRate = {learningRate:F3}; targetDistance = {targetDistance:F3}");

                    var gradient = LossGradient(angles);

                    angles =
                        angles
                        .ToDictionary(
                            item => item.Key,
                            item =>
                            {
                                IKJointAngles updatedAngles = item.Value;
                                updatedAngles.Angles -= learningRate * gradient[item.Key];
                                return updatedAngles;
                            }
                        );

                    if (iterationsPerFrame == ikParams.MaxIterationsPerFrame)
                    {
                        iterationsPerFrame = 0;
                        foreach (var effector in effectors)
                        {
                            foreach (var joint in effector.Joints)
                            {
                                joint.Angles = angles[joint.name];
                            }
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
