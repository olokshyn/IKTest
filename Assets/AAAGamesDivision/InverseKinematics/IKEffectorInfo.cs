using System;
using System.Collections.Generic;
using UnityEngine;

namespace AAAGamesDivision
{
    namespace InverseKinematics
    {
        [Serializable]
        public struct IKEffectorInfo
        {
            public IKEndEffector effector;

            public Transform target;

            [NonSerialized]
            private IKJoint[] joints;

            public IKJoint[] Joints
            {
                get
                {
                    if (joints == null)
                    {
                        joints = effector.Joints;
                    }
                    return joints;
                }
            }

            public IKEffectorInfo(IKEndEffector effector, Transform target = null)
            {
                this.effector = effector;
                this.target = target;
                joints = null;
            }

            public Vector3 ForwardKinematics(Dictionary<string, IKJointAngles> angles)
            {
                var _joints = Joints;
                Quaternion rotation = _joints[0].transform.parent.rotation;
                Vector3 position = _joints[0].transform.position;
                foreach (IKJoint joint in _joints)
                {
                    rotation *= Quaternion.Euler(angles[joint.name].Angles);
                    position += rotation * joint.Arm;
                }
                return position;
            }

            public float DistanceToTarget(Dictionary<string, IKJointAngles> angles)
            {
                return (target.position - ForwardKinematics(angles)).magnitude;
            }
        }
    }
}
