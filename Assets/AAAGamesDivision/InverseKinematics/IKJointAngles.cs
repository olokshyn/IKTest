using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AAAGamesDivision
{
    namespace InverseKinematics
    {
        public struct IKJointAngles
        {
            private Vector3 angles;
            private IKRotationAxes rotationAxes;
            private Vector3 restAngles;
            private Vector3 minAngles;
            private Vector3 maxAngles;

            public IKJointAngles(
                IKRotationAxes rotationAxes,
                Vector3 restAngles,
                Vector3 minAngles,
                Vector3 maxAngles
            )
            {
                angles = restAngles;
                this.rotationAxes = rotationAxes;
                this.restAngles = restAngles;
                this.minAngles = minAngles;
                this.maxAngles = maxAngles;
            }

            public Vector3 Angles
            {
                get => angles;
                set
                {
                    for (int i = 0; i < 3; ++i)
                    {
                        if (rotationAxes[i])
                        {
                            value[i] = Mathf.Clamp(value[i], minAngles[i], maxAngles[i]);
                        }
                        else
                        {
                            value[i] = restAngles[i];
                        }
                    }
                    angles = value;
                }
            }

            public float Tension
            {
                get
                {
                    float tension = 0;
                    for (int i = 0; i < 3; ++i)
                    {
                        float closestLimit =
                            Mathf.Abs(angles[i] - minAngles[i]) < Mathf.Abs(angles[i] - maxAngles[i])
                            ? minAngles[i]
                            : maxAngles[i];
                        tension +=
                            Mathf.Abs(
                                (angles[i] - restAngles[i])
                                / (restAngles[i] - closestLimit)
                            );
                    }
                    return tension / 3f;
                }
            }

            public float this[int i]
            {
                get => angles[i];
                set => angles[i] = value;
            }

            public void ToRest()
            {
                angles = restAngles;
            }

            public void AdjustToConstraints()
            {
                Angles = angles;
            }

            public static implicit operator Vector3(IKJointAngles angles)
            {
                return angles.angles;
            }
        }
    }
}
