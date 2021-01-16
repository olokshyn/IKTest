using System;

namespace AAAGamesDivision
{
    namespace InverseKinematics
    {
        [Serializable]
        public struct IKRotationAxes
        {
            public bool XRotation;
            public bool YRotation;
            public bool ZRotation;

            public IKRotationAxes(
                bool xRotation = true,
                bool yRotation = true,
                bool zRotation = true
            )
            {
                XRotation = xRotation;
                YRotation = yRotation;
                ZRotation = zRotation;
            }

            public bool this[int i]
            {
                get
                {
                    if (i == 0)
                    {
                        return XRotation;
                    }
                    if (i == 1)
                    {
                        return YRotation;
                    }
                    if (i == 2)
                    {
                        return ZRotation;
                    }
                    throw new IndexOutOfRangeException($"Index {i} is out of [0, 3) range");
                }
            }
        }
    }
}
