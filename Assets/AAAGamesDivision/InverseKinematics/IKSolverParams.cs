using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AAAGamesDivision
{
    namespace InverseKinematics
    {
        public struct IKSolverParams
        {
            public float GradientDeltaStep;

            public float MinLearningRate;
            public float MaxLearningRate;

            public float Regularization;

            public float MinTargetDistance;
            public float MaxTargetDistance;

            public int MaxIterationsPerFrame;

            public IKSolverParams(
                float gradientDeltaStep,
                float minLearningRate,
                float maxLearningRate,
                float regularization,
                float minTargetDistance,
                float maxTargetDistance,
                int maxIterationsPerFrame
            )
            {
                GradientDeltaStep = gradientDeltaStep;
                MinLearningRate = minLearningRate;
                MaxLearningRate = maxLearningRate;
                Regularization = regularization;
                MinTargetDistance = minTargetDistance;
                MaxTargetDistance = maxTargetDistance;
                MaxIterationsPerFrame = maxIterationsPerFrame;
            }
        }
    }
}
