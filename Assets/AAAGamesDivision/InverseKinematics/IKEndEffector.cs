using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace AAAGamesDivision
{
    namespace InverseKinematics
    {

        public class IKEndEffector : MonoBehaviour
        {
            public IKJoint[] Joints =>
                GetComponentsInParent(typeof(IKJoint))
                    .Reverse()
                    .Select(x => (IKJoint)x)
                    .ToArray();
        }
    }
}
