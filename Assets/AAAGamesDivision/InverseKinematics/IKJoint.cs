using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AAAGamesDivision
{
    namespace InverseKinematics
    {

        public class IKJoint : MonoBehaviour
        {
            public Vector3 arm;
            public bool xRotation = true;
            public bool yRotation = true;
            public bool zRotation = true;
            public Vector3 restAngles = new Vector3(0f, 0f, 0f);
            public Vector3 minAngles = new Vector3(-90f, -90f, -90f);
            public Vector3 maxAngles = new Vector3(90f, 90f, 90f);

            public IKJointAngles Angles
            {
                get =>
                    new IKJointAngles(
                        new IKRotationAxes(xRotation, yRotation, zRotation),
                        restAngles,
                        minAngles,
                        maxAngles
                    );
                set
                {
                    transform.localEulerAngles = value.Angles;
                }
            }

            void Reset()
            {
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
