using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AAAGamesDivision
{
    namespace InverseKinematics
    {
        public class IKJoint : MonoBehaviour
        {
            [SerializeField]
            private Vector3 arm;

            [SerializeField]
            private IKJointAngles angles;

            public IKJointAngles Angles
            {
                // IKJointAngles is a struct so it's returned by value
                get => angles;
                set
                {
                    transform.localEulerAngles = value.Angles;
                    angles = value;
                }
            }

            public Vector3 Arm => arm;

            void Reset()
            {
                Vector3 localRotation = transform.localEulerAngles;
                for (int i = 0; i < 3; ++i)
                {
                    localRotation[i] = Mathf.Round(
                        Mathf.DeltaAngle(0, localRotation[i])
                    );
                }
                angles = new IKJointAngles(
                    rotationAxes: new IKRotationAxes(
                        xRotation: true,
                        yRotation: true,
                        zRotation: true
                    ),
                    restAngles: localRotation,
                    minAngles: localRotation - 90f * Vector3.one,
                    maxAngles: localRotation + 90f * Vector3.one
                );

                IKJoint[] joints = GetComponentsInChildren<IKJoint>();
                if (joints.Length >= 2)
                {
                    IKJoint childJoint = joints[1];
                    arm = childJoint.transform.localPosition;
                }
                else
                {
                    IKEndEffector effector = GetComponentInChildren<IKEndEffector>();
                    if (effector)
                    {
                        arm = effector.transform.localPosition;
                    }
                }
            }
        }
    }
}
