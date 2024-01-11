using Joycon4CS;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Xml.Linq;

namespace JoyconPlugin.Fusion
{
    public class FusionMath
    {
        public struct FusionVectorAxis
        {
            public float x;
            public float y;
            public float z;
        }

        /**
    * @brief 3D vector.
*/
        public struct FusionVector
        {
            public float[] array;
            public FusionVectorAxis axis;

            public FusionVector(float x, float y, float z)
            {
                array = new float[3];
                axis = new FusionVectorAxis()
                {
                    x = x,
                    y = y,
                    z = z
                };
            }
        }

        public struct FusionQuaternionElement
        {
            public float w;
            public float x;
            public float y;
            public float z;
        }

        /**
         * @brief Quaternion.
         */
        public struct FusionQuaternion
        {
            public float[] array;
            public FusionQuaternionElement element;
            public FusionQuaternion(float w, float x, float y, float z)
            {
                array = new float[4];
                element = new FusionQuaternionElement()
                {
                    w = w,
                    x = x,
                    y = y,
                    z = z
                };
            }
        }

        public struct FusionMatrixElement
        {
            public float xx;
            public float xy;
            public float xz;
            public float yx;
            public float yy;
            public float yz;
            public float zx;
            public float zy;
            public float zz;
        }
        /**
         * @brief 3x3 matrix in row-major order.
         * See http://en.wikipedia.org/wiki/Row-major_order
         */
        public struct FusionMatrix
        {
            public float[][] array;
            public FusionMatrixElement element;
            public FusionMatrix(float xx,
            float xy,
            float xz,
            float yx,
            float yy,
            float yz,
            float zx,
            float zy,
            float zz)
            {
                array = new float[][] { new float[] { 0f } };
                element = new FusionMatrixElement();
                element.xx = xx;
                element.xy = xy;
                element.xz = xz;
                element.yx = yx;
                element.yy = yy;
                element.yz = yz;
                element.zx = zx;
                element.zy = zy;
                element.zz = zz;
            }
        }

        public struct FusionEulerAngle
        {
            public float roll;
            public float pitch;
            public float yaw;
        }

        /**
         * @brief Euler angles.  Roll, pitch, and yaw correspond to rotations around
         * X, Y, and Z respectively.
         */
        public struct FusionEuler
        {
            public float[] array;

            public FusionEulerAngle angle;

            public FusionEuler(float roll, float pitch, float yaw)
            {
                array = new float[3];
                angle = new FusionEulerAngle()
                {
                    roll = roll,
                    pitch = pitch,
                    yaw = yaw
                };
            }
        };

        /**
         * @brief Vector of zeros.
         */
        public static readonly FusionVector FUSION_VECTOR_ZERO = new FusionVector(0.0f, 0.0f, 0.0f);

        /**
         * @brief Vector of ones.
         */
        public static readonly FusionVector FUSION_VECTOR_ONES = new FusionVector(1.0f, 1.0f, 1.0f);

        /**
         * @brief Identity quaternion.
         */
        public static readonly FusionQuaternion FUSION_IDENTITY_QUATERNION = new FusionQuaternion(1.0f, 0.0f, 0.0f, 0.0f);

        /**
         * @brief Identity matrix.
         */
        public static readonly FusionMatrix FUSION_IDENTITY_MATRIX = new FusionMatrix(1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f);

        /**
         * @brief Euler angles of zero.
         */
        public static readonly FusionEuler FUSION_EULER_ZERO = new FusionEuler(0.0f, 0.0f, 0.0f);

        /**
         * @brief Pi. May not be defined in math.h.
         */
        public const double M_PI = 3.14159265358979323846;

        /**
         * @brief Include this definition or add as a preprocessor definition to use
         * normal square root operations.
         */
        //#define FUSION_USE_NORMAL_SQRT

        //------------------------------------------------------------------------------
        // functions - Degrees and radians conversion

        /**
         * @brief Converts degrees to radians.
         * @param degrees Degrees.
         * @return Radians.
         */
        public static float FusionDegreesToRadians(float degrees)
        {
            return degrees * ((float)M_PI / 180.0f);
        }

        /**
         * @brief Converts radians to degrees.
         * @param radians Radians.
         * @return Degrees.
         */
        public static float FusionRadiansToDegrees(float radians)
        {
            return radians * (180.0f / (float)M_PI);
        }

        //------------------------------------------------------------------------------
        // functions - Arc sine

        /**
         * @brief Returns the arc sine of the value.
         * @param value Value.
         * @return Arc sine of the value.
         */
        public static float FusionAsin(float value)
        {
            if (value <= -1.0f)
            {
                return (float)M_PI / -2.0f;
            }
            if (value >= 1.0f)
            {
                return (float)M_PI / 2.0f;
            }
            return (float)Math.Asin(value);
        }

        //------------------------------------------------------------------------------
        // functions - Vector operations

        /**
         * @brief Returns true if the vector is zero.
         * @param vector Vector.
         * @return True if the vector is zero.
         */
        public static bool FusionVectorIsZero(FusionVector vector)
        {
            return (vector.axis.x == 0.0f) && (vector.axis.y == 0.0f) && (vector.axis.z == 0.0f);
        }

        /**
         * @brief Returns the sum of two vectors.
         * @param vectorA Vector A.
         * @param vectorB Vector B.
         * @return Sum of two vectors.
         */
        public static FusionVector FusionVectorAdd(FusionVector vectorA, FusionVector vectorB)
        {
            FusionVector result = new FusionVector()
            {
                axis = new FusionVectorAxis()
                {
                    x = vectorA.axis.x + vectorB.axis.x,
                    y = vectorA.axis.y + vectorB.axis.y,
                    z = vectorA.axis.z + vectorB.axis.z,
                }
            };
            return result;
        }

        /**
         * @brief Returns vector B subtracted from vector A.
         * @param vectorA Vector A.
         * @param vectorB Vector B.
         * @return Vector B subtracted from vector A.
         */
        public static FusionVector FusionVectorSubtract(FusionVector vectorA, FusionVector vectorB)
        {
            FusionVector result = new FusionVector()
            {
                axis = new FusionVectorAxis()
                {
                    x = vectorA.axis.x - vectorB.axis.x,
                    y = vectorA.axis.y - vectorB.axis.y,
                    z = vectorA.axis.z - vectorB.axis.z,
                }
            };
            return result;
        }

        /**
         * @brief Returns the sum of the elements.
         * @param vector Vector.
         * @return Sum of the elements.
         */
        public static float FusionVectorSum(FusionVector vector)
        {
            return vector.axis.x + vector.axis.y + vector.axis.z;
        }

        /**
         * @brief Returns the multiplication of a vector by a scalar.
         * @param vector Vector.
         * @param scalar Scalar.
         * @return Multiplication of a vector by a scalar.
         */
        public static FusionVector FusionVectorMultiplyScalar(FusionVector vector, float scalar)
        {
            FusionVector result = new FusionVector()
            {
                axis = new FusionVectorAxis()
                {
                    x = vector.axis.x * scalar,
                    y = vector.axis.y * scalar,
                    z = vector.axis.z * scalar,
                }
            };
            return result;
        }

        /**
         * @brief Calculates the Hadamard product (element-wise multiplication).
         * @param vectorA Vector A.
         * @param vectorB Vector B.
         * @return Hadamard product.
         */
        public static FusionVector FusionVectorHadamardProduct(FusionVector vectorA, FusionVector vectorB)
        {
            FusionVector result = new FusionVector()
            {
                axis = new FusionVectorAxis()
                {
                    x = vectorA.axis.x * vectorB.axis.x,
                    y = vectorA.axis.y * vectorB.axis.y,
                    z = vectorA.axis.z * vectorB.axis.z,
                }
            };
            return result;
        }

        /**
         * @brief Returns the cross product.
         * @param vectorA Vector A.
         * @param vectorB Vector B.
         * @return Cross product.
         */
        public static FusionVector FusionVectorCrossProduct(FusionVector vectorA, FusionVector vectorB)
        {
            FusionVectorAxis A = vectorA.axis;
            FusionVectorAxis B = vectorB.axis;
            FusionVector result = new FusionVector()
            {
                axis = new FusionVectorAxis()
                {
                    x = A.y * B.z - A.z * B.y,
                    y = A.z * B.x - A.x * B.z,
                    z = A.x * B.y - A.y * B.x,
                }
            };
            return result;
        }

        /**
         * @brief Returns the dot product.
         * @param vectorA Vector A.
         * @param vectorB Vector B.
         * @return Dot product.
         */
        public static float FusionVectorDotProduct(FusionVector vectorA, FusionVector vectorB)
        {
            return FusionVectorSum(FusionVectorHadamardProduct(vectorA, vectorB));
        }

        /**
         * @brief Returns the vector magnitude squared.
         * @param vector Vector.
         * @return Vector magnitude squared.
         */
        public static float FusionVectorMagnitudeSquared(FusionVector vector)
        {
            return FusionVectorSum(FusionVectorHadamardProduct(vector, vector));
        }

        /**
         * @brief Returns the vector magnitude.
         * @param vector Vector.
         * @return Vector magnitude.
         */
        public static float FusionVectorMagnitude(FusionVector vector)
        {
            return (float)Math.Sqrt(FusionVectorMagnitudeSquared(vector));
        }

        /**
         * @brief Returns the normalised vector.
         * @param vector Vector.
         * @return Normalised vector.
         */
        public static FusionVector FusionVectorNormalise(FusionVector vector)
        {
            float magnitudeReciprocal = 1.0f / (float)Math.Sqrt(FusionVectorMagnitudeSquared(vector));
            return FusionVectorMultiplyScalar(vector, magnitudeReciprocal);
        }

        //------------------------------------------------------------------------------
        // functions - Quaternion operations

        /**
         * @brief Returns the sum of two quaternions.
         * @param quaternionA Quaternion A.
         * @param quaternionB Quaternion B.
         * @return Sum of two quaternions.
         */
        public static FusionQuaternion FusionQuaternionAdd(FusionQuaternion quaternionA, FusionQuaternion quaternionB)
        {
            FusionQuaternion result = new FusionQuaternion()
            {
                element = new FusionQuaternionElement()
                {
                    w = quaternionA.element.w + quaternionB.element.w,
                    x = quaternionA.element.x + quaternionB.element.x,
                    y = quaternionA.element.y + quaternionB.element.y,
                    z = quaternionA.element.z + quaternionB.element.z,
                }
            };
            return result;
        }

        /**
         * @brief Returns the multiplication of two quaternions.
         * @param quaternionA Quaternion A (to be post-multiplied).
         * @param quaternionB Quaternion B (to be pre-multiplied).
         * @return Multiplication of two quaternions.
         */
        public static FusionQuaternion FusionQuaternionMultiply(FusionQuaternion quaternionA, FusionQuaternion quaternionB)
        {
            FusionQuaternionElement A = quaternionA.element;
            FusionQuaternionElement B = quaternionB.element;
            FusionQuaternion result = new FusionQuaternion()
            {
                element = new FusionQuaternionElement()
                {
                    w = A.w * B.w - A.x * B.x - A.y * B.y - A.z * B.z,
                    x = A.w * B.x + A.x * B.w + A.y * B.z - A.z * B.y,
                    y = A.w * B.y - A.x * B.z + A.y * B.w + A.z * B.x,
                    z = A.w * B.z + A.x * B.y - A.y * B.x + A.z * B.w,
                }
            };
            return result;
        }

        /**
         * @brief Returns the multiplication of a quaternion with a vector.  This is a
         * normal quaternion multiplication where the vector is treated a
         * quaternion with a W element value of zero.  The quaternion is post-
         * multiplied by the vector.
         * @param quaternion Quaternion.
         * @param vector Vector.
         * @return Multiplication of a quaternion with a vector.
         */
        public static FusionQuaternion FusionQuaternionMultiplyVector(FusionQuaternion quaternion, FusionVector vector)
        {
            FusionQuaternionElement Q = quaternion.element;
            FusionVectorAxis V = vector.axis;
            FusionQuaternion result = new FusionQuaternion()
            {
                element = new FusionQuaternionElement()
                {
                    w = -Q.x * V.x - Q.y * V.y - Q.z * V.z,
                    x = Q.w * V.x + Q.y * V.z - Q.z * V.y,
                    y = Q.w * V.y - Q.x * V.z + Q.z * V.x,
                    z = Q.w * V.z + Q.x * V.y - Q.y * V.x,
                }
            };
            return result;
        }

        /**
         * @brief Returns the normalised quaternion.
         * @param quaternion Quaternion.
         * @return Normalised quaternion.
         */
        public static FusionQuaternion FusionQuaternionNormalise(FusionQuaternion quaternion)
        {
            FusionQuaternionElement Q = quaternion.element;
            float sqrt = (float)Math.Sqrt(Q.w * Q.w + Q.x * Q.x + Q.y * Q.y + Q.z * Q.z);
            float magnitudeReciprocal = sqrt > 0 ? (float)(1.0f / sqrt) : 0;
            FusionQuaternion result = new FusionQuaternion()
            {
                element = new FusionQuaternionElement()
                {
                    w = Q.w * magnitudeReciprocal,
                    x = Q.x * magnitudeReciprocal,
                    y = Q.y * magnitudeReciprocal,
                    z = Q.z * magnitudeReciprocal,
                }
            };
            return result;
        }

        //------------------------------------------------------------------------------
        // functions - Matrix operations

        /**
         * @brief Returns the multiplication of a matrix with a vector.
         * @param matrix Matrix.
         * @param vector Vector.
         * @return Multiplication of a matrix with a vector.
         */
        public static FusionVector FusionMatrixMultiplyVector(FusionMatrix matrix, FusionVector vector)
        {
            FusionMatrixElement R = matrix.element;
            FusionVector result = new FusionVector()
            {
                axis = new FusionVectorAxis()
                {
                    x = R.xx * vector.axis.x + R.xy * vector.axis.y + R.xz * vector.axis.z,
                    y = R.yx * vector.axis.x + R.yy * vector.axis.y + R.yz * vector.axis.z,
                    z = R.zx * vector.axis.x + R.zy * vector.axis.y + R.zz * vector.axis.z,
                }
            };
            return result;
        }

        //------------------------------------------------------------------------------
        // functions - Conversion operations

        /**
         * @brief Converts a quaternion to a rotation matrix.
         * @param quaternion Quaternion.
         * @return Rotation matrix.
         */
        public static FusionMatrix FusionQuaternionToMatrix(FusionQuaternion quaternion)
        {
            FusionQuaternionElement Q = quaternion.element;
            float qwqw = Q.w * Q.w; // calculate common terms to avoid repeated operations
            float qwqx = Q.w * Q.x;
            float qwqy = Q.w * Q.y;
            float qwqz = Q.w * Q.z;
            float qxqy = Q.x * Q.y;
            float qxqz = Q.x * Q.z;
            float qyqz = Q.y * Q.z;
            FusionMatrix matrix = new FusionMatrix()
            {
                element = new FusionMatrixElement()
                {
                    xx = 2.0f * (qwqw - 0.5f + Q.x * Q.x),
                    xy = 2.0f * (qxqy - qwqz),
                    xz = 2.0f * (qxqz + qwqy),
                    yx = 2.0f * (qxqy + qwqz),
                    yy = 2.0f * (qwqw - 0.5f + Q.y * Q.y),
                    yz = 2.0f * (qyqz - qwqx),
                    zx = 2.0f * (qxqz - qwqy),
                    zy = 2.0f * (qyqz + qwqx),
                    zz = 2.0f * (qwqw - 0.5f + Q.z * Q.z),
                }
            };
            return matrix;
        }

        /**
         * @brief Converts a quaternion to ZYX Euler angles in degrees.
         * @param quaternion Quaternion.
         * @return Euler angles in degrees.
         */
        public static FusionEuler FusionQuaternionToEuler(FusionQuaternion quaternion)
        {
            FusionQuaternionElement Q = quaternion.element;
            float halfMinusQySquared = 0.5f - Q.y * Q.y; // calculate common terms to avoid repeated operations
            FusionEuler euler = new FusionEuler()
            {
                angle = new FusionEulerAngle()
                {
                    roll = FusionRadiansToDegrees((float)Math.Atan2(Q.w * Q.x + Q.y * Q.z, halfMinusQySquared - Q.x * Q.x)),
                    pitch = FusionRadiansToDegrees(FusionAsin(2.0f * (Q.w * Q.y - Q.z * Q.x))),
                    yaw = FusionRadiansToDegrees((float)Math.Atan2(Q.w * Q.z + Q.x * Q.y, halfMinusQySquared - Q.z * Q.z)),
                }
            };
            return euler;
        }
    }
}
