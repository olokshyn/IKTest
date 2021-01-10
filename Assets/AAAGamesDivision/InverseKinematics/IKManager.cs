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
            public Transform ikTarget;

            public float deltaStep = 0.1f;
            public float minLearningRate = 0.1f;
            public float maxLearningRate = 5f;
            public float minTargetDistance = 0.01f;
            public float maxTargetDistance = 0.1f;

            public int maxIterationsPerFrame = 20;

            public float regularization = 0.8f;

            private IKSolver solver;

            void Reset()
            {
                joints = GetComponentsInChildren(typeof(IKJoint))
                    .Select(x => (IKJoint)x)
                    .ToArray();
                ikTarget = GameObject.Find("Controller").transform;
            }

            void Start()
            {
                IKSolverParams ikParams = new IKSolverParams(
                    gradientDeltaStep: deltaStep,
                    minLearningRate: minLearningRate,
                    maxLearningRate: maxLearningRate,
                    regularization: regularization,
                    minTargetDistance: minTargetDistance,
                    maxTargetDistance: maxTargetDistance,
                    maxIterationsPerFrame: maxIterationsPerFrame
                );
                solver = new IKSolver(joints, ikTarget, ikParams);
                StartCoroutine(solver.StartSolver());
            }
        }
    }
}
