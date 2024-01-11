using JoyconPlugin.Fusion;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using static JoyconPlugin.Fusion.FusionMath;

namespace JoyconPlugin.Fusion
{
    public class FusionAhrs
    {
        //------------------------------------------------------------------------------
        // Definitions

        /**
         * @brief this algorithm settings.
         */
        public struct FusionAhrsSettings
        {
            public FusionConvention convention;
            public float gain;
            public float gyroscopeRange;
            public float accelerationRejection;
            public float magneticRejection;
            public uint recoveryTriggerPeriod;
        }



        /**
         * @brief this algorithm structure.  Structure members are used internally and
         * must not be accessed by the application.
         */
        FusionAhrsSettings settings;
        FusionQuaternion quaternion;
        FusionVector accelerometer;
        bool initialising;
        float rampedGain;
        float rampedGainStep;
        bool angularRateRecovery;
        FusionVector halfAccelerometerFeedback;
        FusionVector halfMagnetometerFeedback;
        bool accelerometerIgnored;
        int accelerationRecoveryTrigger;
        int accelerationRecoveryTimeout;
        bool magnetometerIgnored;
        int magneticRecoveryTrigger;
        int magneticRecoveryTimeout;

        /**
         * @brief this algorithm internal states.
         */
        public struct FusionAhrsInternalStates
        {
            public float accelerationError;
            public bool accelerometerIgnored;
            public float accelerationRecoveryTrigger;
            public float magneticError;
            public bool magnetometerIgnored;
            public float magneticRecoveryTrigger;
        };

        /**
         * @brief this algorithm flags.
         */
        public struct FusionAhrsFlags
        {
            public bool initialising;
            public bool angularRateRecovery;
            public bool accelerationRecovery;
            public bool magneticRecovery;
        };

        const float INITIAL_GAIN = 10.0f;

        const float INITIALISATION_PERIOD = 3.0f;

        const float FLT_MAX = float.MaxValue;

        //------------------------------------------------------------------------------
        // Functions

        /**
         * @brief Initialises the this algorithm structure.
         * @param this this algorithm structure.
         */
        public FusionAhrs()
        {
            FusionAhrsSettings settings = new FusionAhrsSettings()
            {
                convention = FusionConvention.FusionConventionNwu,
                gain = 0.5f,
                gyroscopeRange = 0.0f,
                accelerationRejection = 90.0f,
                magneticRejection = 90.0f,
                recoveryTriggerPeriod = 0,
            };
            FusionAhrsSetSettings(settings);
            FusionAhrsReset();
        }

        /**
         * @brief Resets the this algorithm.  This is equivalent to reinitialising the
         * algorithm while maintaining the current settings.
         * @param this this algorithm structure.
         */
        public void FusionAhrsReset()
        {
            this.quaternion = FUSION_IDENTITY_QUATERNION;
            this.accelerometer = FUSION_VECTOR_ZERO;
            this.initialising = true;
            this.rampedGain = INITIAL_GAIN;
            this.angularRateRecovery = false;
            this.halfAccelerometerFeedback = FUSION_VECTOR_ZERO;
            this.halfMagnetometerFeedback = FUSION_VECTOR_ZERO;
            this.accelerometerIgnored = false;
            this.accelerationRecoveryTrigger = 0;
            this.accelerationRecoveryTimeout = (int)this.settings.recoveryTriggerPeriod;
            this.magnetometerIgnored = false;
            this.magneticRecoveryTrigger = 0;
            this.magneticRecoveryTimeout = (int)this.settings.recoveryTriggerPeriod;
        }

        /**
         * @brief Sets the this algorithm settings.
         * @param this this algorithm structure.
         * @param settings Settings.
         */
        public void FusionAhrsSetSettings(FusionAhrsSettings settings)
        {
            this.settings.convention = settings.convention;
            this.settings.gain = settings.gain;
            this.settings.gyroscopeRange = settings.gyroscopeRange == 0.0f ? FLT_MAX : 0.98f * settings.gyroscopeRange;
            this.settings.accelerationRejection = (float)(settings.accelerationRejection == 0.0f ? FLT_MAX : Math.Pow(0.5f * Math.Sin(FusionDegreesToRadians(settings.accelerationRejection)), 2));
            this.settings.magneticRejection = (float)(settings.magneticRejection == 0.0f ? FLT_MAX : Math.Pow(0.5f * Math.Sin(FusionDegreesToRadians(settings.magneticRejection)), 2));
            this.settings.recoveryTriggerPeriod = settings.recoveryTriggerPeriod;
            this.accelerationRecoveryTimeout = (int)this.settings.recoveryTriggerPeriod;
            this.magneticRecoveryTimeout = (int)this.settings.recoveryTriggerPeriod;
            if ((settings.gain == 0.0f) || (settings.recoveryTriggerPeriod == 0))
            { // disable acceleration and magnetic rejection features if gain is zero
                this.settings.accelerationRejection = FLT_MAX;
                this.settings.magneticRejection = FLT_MAX;
            }
            if (this.initialising == false)
            {
                this.rampedGain = this.settings.gain;
            }
            this.rampedGainStep = (INITIAL_GAIN - this.settings.gain) / INITIALISATION_PERIOD;
        }

        /**
         * @brief Updates the this algorithm using the gyroscope, accelerometer, and
         * magnetometer measurements.
         * @param this this algorithm structure.
         * @param gyroscope Gyroscope measurement in degrees per second.
         * @param accelerometer Accelerometer measurement in g.
         * @param magnetometer Magnetometer measurement in arbitrary units.
         * @param deltaTime Delta time in seconds.
         */
        public void FusionAhrsUpdate(FusionVector gyroscope, FusionVector accelerometer, FusionVector magnetometer, float deltaTime)
        {
            FusionQuaternionElement Q = this.quaternion.element;

            // Store accelerometer
            this.accelerometer = accelerometer;

            // Reinitialise if gyroscope range exceeded
            if (((float)Math.Abs(gyroscope.axis.x) > this.settings.gyroscopeRange) || ((float)Math.Abs(gyroscope.axis.y) > this.settings.gyroscopeRange) || ((float)Math.Abs(gyroscope.axis.z) > this.settings.gyroscopeRange))
            {
                FusionQuaternion quaternion = this.quaternion;
                FusionAhrsReset();
                this.quaternion = quaternion;
                this.angularRateRecovery = true;
            }

            // Ramp down gain during initialisation
            if (this.initialising == true)
            {
                this.rampedGain -= this.rampedGainStep * deltaTime;
                if ((this.rampedGain < this.settings.gain) || (this.settings.gain == 0.0f))
                {
                    this.rampedGain = this.settings.gain;
                    this.initialising = false;
                    this.angularRateRecovery = false;
                }
            }

            // Calculate direction of gravity indicated by algorithm
            FusionVector halfGravity = HalfGravity();

            // Calculate accelerometer feedback
            FusionVector halfAccelerometerFeedback = FUSION_VECTOR_ZERO;
            this.accelerometerIgnored = true;
            if (FusionVectorIsZero(accelerometer) == false)
            {

                // Calculate accelerometer feedback scaled by 0.5
                this.halfAccelerometerFeedback = Feedback(FusionVectorNormalise(accelerometer), halfGravity);

                // Don't ignore accelerometer if acceleration error below threshold
                if ((this.initialising == true) || ((FusionVectorMagnitudeSquared(this.halfAccelerometerFeedback) <= this.settings.accelerationRejection)))
                {
                    this.accelerometerIgnored = false;
                    this.accelerationRecoveryTrigger -= 9;
                }
                else
                {
                    this.accelerationRecoveryTrigger += 1;
                }

                // Don't ignore accelerometer during acceleration recovery
                if (this.accelerationRecoveryTrigger > this.accelerationRecoveryTimeout)
                {
                    this.accelerationRecoveryTimeout = 0;
                    this.accelerometerIgnored = false;
                }
                else
                {
                    this.accelerationRecoveryTimeout = (int)this.settings.recoveryTriggerPeriod;
                }
                this.accelerationRecoveryTrigger = Clamp(this.accelerationRecoveryTrigger, 0, (int)this.settings.recoveryTriggerPeriod);

                // Apply accelerometer feedback
                if (this.accelerometerIgnored == false)
                {
                    halfAccelerometerFeedback = this.halfAccelerometerFeedback;
                }
            }

            // Calculate magnetometer feedback
            FusionVector halfMagnetometerFeedback = FUSION_VECTOR_ZERO;
            this.magnetometerIgnored = true;
            if (FusionVectorIsZero(magnetometer) == false)
            {

                // Calculate direction of magnetic field indicated by algorithm
                FusionVector halfMagnetic = HalfMagnetic();

                // Calculate magnetometer feedback scaled by 0.5
                this.halfMagnetometerFeedback = Feedback(FusionVectorNormalise(FusionVectorCrossProduct(halfGravity, magnetometer)), halfMagnetic);

                // Don't ignore magnetometer if magnetic error below threshold
                if ((this.initialising == true) || ((FusionVectorMagnitudeSquared(this.halfMagnetometerFeedback) <= this.settings.magneticRejection)))
                {
                    this.magnetometerIgnored = false;
                    this.magneticRecoveryTrigger -= 9;
                }
                else
                {
                    this.magneticRecoveryTrigger += 1;
                }

                // Don't ignore magnetometer during magnetic recovery
                if (this.magneticRecoveryTrigger > this.magneticRecoveryTimeout)
                {
                    this.magneticRecoveryTimeout = 0;
                    this.magnetometerIgnored = false;
                }
                else
                {
                    this.magneticRecoveryTimeout = (int)this.settings.recoveryTriggerPeriod;
                }
                this.magneticRecoveryTrigger = Clamp(this.magneticRecoveryTrigger, 0, (int)this.settings.recoveryTriggerPeriod);

                // Apply magnetometer feedback
                if (this.magnetometerIgnored == false)
                {
                    halfMagnetometerFeedback = this.halfMagnetometerFeedback;
                }
            }

            // Convert gyroscope to radians per second scaled by 0.5
            FusionVector halfGyroscope = FusionVectorMultiplyScalar(gyroscope, FusionDegreesToRadians(0.5f));

            // Apply feedback to gyroscope
            FusionVector adjustedHalfGyroscope = FusionVectorAdd(halfGyroscope, FusionVectorMultiplyScalar(FusionVectorAdd(halfAccelerometerFeedback, halfMagnetometerFeedback), this.rampedGain));

            // Integrate rate of change of quaternion
            this.quaternion = FusionQuaternionAdd(this.quaternion, FusionQuaternionMultiplyVector(this.quaternion, FusionVectorMultiplyScalar(adjustedHalfGyroscope, deltaTime)));

            // Normalise quaternion
            this.quaternion = FusionQuaternionNormalise(this.quaternion);
        }

        /**
         * @brief Returns the direction of gravity scaled by 0.5.
         * @param this this algorithm structure.
         * @return Direction of gravity scaled by 0.5.
         */
        public FusionVector HalfGravity()
        {
            FusionQuaternionElement Q = this.quaternion.element;
            switch (this.settings.convention)
            {
                case FusionConvention.FusionConventionNwu:
                case FusionConvention.FusionConventionEnu:
                    {
                        FusionVector halfGravity = new FusionVector()
                        {
                            axis = new FusionVectorAxis()
                            {
                                x = Q.x * Q.z - Q.w * Q.y,
                                y = Q.y * Q.z + Q.w * Q.x,
                                z = Q.w * Q.w - 0.5f + Q.z * Q.z,
                            }
                        }; // third column of transposed rotation matrix scaled by 0.5
                        return halfGravity;
                    }
                case FusionConvention.FusionConventionNed:
                    {
                        FusionVector halfGravity = new FusionVector()
                        {
                            axis = new FusionVectorAxis()
                            {
                                x = Q.w * Q.y - Q.x * Q.z,
                                y = -1.0f * (Q.y * Q.z + Q.w * Q.x),
                                z = 0.5f - Q.w * Q.w - Q.z * Q.z,
                            }
                        }; // third column of transposed rotation matrix scaled by -0.5
                        return halfGravity;
                    }
            }
            return FUSION_VECTOR_ZERO; // apublic void compiler warning
        }

        /**
         * @brief Returns the direction of the magnetic field scaled by 0.5.
         * @param this this algorithm structure.
         * @return Direction of the magnetic field scaled by 0.5.
         */
        public FusionVector HalfMagnetic()
        {
            FusionQuaternionElement Q = this.quaternion.element;
            switch (this.settings.convention)
            {
                case FusionConvention.FusionConventionNwu:
                    {
                        FusionVector halfMagnetic = new FusionVector()
                        {
                            axis = new FusionVectorAxis()
                            {
                                x = Q.x * Q.y + Q.w * Q.z,
                                y = Q.w * Q.w - 0.5f + Q.y * Q.y,
                                z = Q.y * Q.z - Q.w * Q.x,
                            }
                        }; // second column of transposed rotation matrix scaled by 0.5
                        return halfMagnetic;
                    }
                case FusionConvention.FusionConventionEnu:
                    {
                        FusionVector halfMagnetic = new FusionVector()
                        {
                            axis = new FusionVectorAxis()
                            {
                                x = 0.5f - Q.w * Q.w - Q.x * Q.x,
                                y = Q.w * Q.z - Q.x * Q.y,
                                z = -1.0f * (Q.x * Q.z + Q.w * Q.y),
                            }
                        }; // first column of transposed rotation matrix scaled by -0.5
                        return halfMagnetic;
                    }
                case FusionConvention.FusionConventionNed:
                    {
                        FusionVector halfMagnetic = new FusionVector()
                        {
                            axis = new FusionVectorAxis()
                            {
                                x = -1.0f * (Q.x * Q.y + Q.w * Q.z),
                                y = 0.5f - Q.w * Q.w - Q.y * Q.y,
                                z = Q.w * Q.x - Q.y * Q.z,
                            }
                        }; // second column of transposed rotation matrix scaled by -0.5
                        return halfMagnetic;
                    }
            }
            return FUSION_VECTOR_ZERO; // apublic void compiler warning
        }

        /**
         * @brief Returns the feedback.
         * @param sensor Sensor.
         * @param reference Reference.
         * @return Feedback.
         */
        public FusionVector Feedback(FusionVector sensor, FusionVector reference)
        {
            if (FusionVectorDotProduct(sensor, reference) < 0.0f)
            { // if error is >90 degrees
                return FusionVectorNormalise(FusionVectorCrossProduct(sensor, reference));
            }
            return FusionVectorCrossProduct(sensor, reference);
        }

        /**
         * @brief Returns a value limited to maximum and minimum.
         * @param value Value.
         * @param min Minimum value.
         * @param max Maximum value.
         * @return Value limited to maximum and minimum.
         */
        public int Clamp(int value, int min, int max)
        {
            if (value < min)
            {
                return min;
            }
            if (value > max)
            {
                return max;
            }
            return value;
        }

        /**
         * @brief Updates the this algorithm using the gyroscope and accelerometer
         * measurements only.
         * @param this this algorithm structure.
         * @param gyroscope Gyroscope measurement in degrees per second.
         * @param accelerometer Accelerometer measurement in g.
         * @param deltaTime Delta time in seconds.
         */
        public void FusionAhrsUpdateNoMagnetometer(FusionVector gyroscope, FusionVector accelerometer, float deltaTime)
        {

            // Update this algorithm
            FusionAhrsUpdate(gyroscope, accelerometer, FUSION_VECTOR_ZERO, deltaTime);

            // Zero heading during initialisation
            if (this.initialising == true)
            {
                FusionAhrsSetHeading(0.0f);
            }
        }

        /**
         * @brief Updates the this algorithm using the gyroscope, accelerometer, and
         * heading measurements.
         * @param this this algorithm structure.
         * @param gyroscope Gyroscope measurement in degrees per second.
         * @param accelerometer Accelerometer measurement in g.
         * @param heading Heading measurement in degrees.
         * @param deltaTime Delta time in seconds.
         */
        public void FusionAhrsUpdateExternalHeading(FusionVector gyroscope, FusionVector accelerometer, float heading, float deltaTime)
        {
            FusionQuaternionElement Q = this.quaternion.element;

            // Calculate roll
            float roll = (float)Math.Atan2(Q.w * Q.x + Q.y * Q.z, 0.5f - Q.y * Q.y - Q.x * Q.x);

            // Calculate magnetometer
            float headingRadians = FusionDegreesToRadians(heading);
            float sinHeadingRadians = (float)Math.Sin(headingRadians);
            FusionVector magnetometer = new FusionVector()
            {
                axis = new FusionVectorAxis()
                {
                    x = (float)Math.Cos(headingRadians),
                    y = -1.0f * (float)Math.Cos(roll) * sinHeadingRadians,
                    z = sinHeadingRadians * (float)Math.Sin(roll),
                }
            };

            // Update this algorithm
            FusionAhrsUpdate(gyroscope, accelerometer, magnetometer, deltaTime);
        }

        /**
         * @brief Returns the quaternion describing the sensor relative to the Earth.
         * @param this this algorithm structure.
         * @return Quaternion describing the sensor relative to the Earth.
         */
        public FusionQuaternion FusionAhrsGetQuaternion()
        {
            return this.quaternion;
        }

        /**
         * @brief Sets the quaternion describing the sensor relative to the Earth.
         * @param this this algorithm structure.
         * @param quaternion Quaternion describing the sensor relative to the Earth.
         */
        public void FusionAhrsSetQuaternion(FusionQuaternion quaternion)
        {
            this.quaternion = quaternion;
        }

        /**
         * @brief Returns the linear acceleration measurement equal to the accelerometer
         * measurement with the 1 g of gravity removed.
         * @param this this algorithm structure.
         * @return Linear acceleration measurement in g.
         */
        public FusionVector FusionAhrsGetLinearAcceleration()
        {
            FusionQuaternionElement Q = this.quaternion.element;

            // Calculate gravity in the sensor coordinate frame
            FusionVector gravity = new FusionVector()
            {
                axis = new FusionVectorAxis()
                {
                    x = 2.0f * (Q.x * Q.z - Q.w * Q.y),
                    y = 2.0f * (Q.y * Q.z + Q.w * Q.x),
                    z = 2.0f * (Q.w * Q.w - 0.5f + Q.z * Q.z),
                }
            }; // third column of transposed rotation matrix

            // Remove gravity from accelerometer measurement
            switch (this.settings.convention)
            {
                case FusionConvention.FusionConventionNwu:
                case FusionConvention.FusionConventionEnu:
                    {
                        return FusionVectorSubtract(this.accelerometer, gravity);
                    }
                case FusionConvention.FusionConventionNed:
                    {
                        return FusionVectorAdd(this.accelerometer, gravity);
                    }
            }
            return FUSION_VECTOR_ZERO; // apublic void compiler warning
        }

        /**
         * @brief Returns the Earth acceleration measurement equal to accelerometer
         * measurement in the Earth coordinate frame with the 1 g of gravity removed.
         * @param this this algorithm structure.
         * @return Earth acceleration measurement in g.
         */
        public FusionVector FusionAhrsGetEarthAcceleration()
        {
            FusionQuaternionElement Q = this.quaternion.element;
            FusionVectorAxis A = this.accelerometer.axis;

            // Calculate accelerometer measurement in the Earth coordinate frame
            float qwqw = Q.w * Q.w; // calculate common terms to apublic void repeated operations
            float qwqx = Q.w * Q.x;
            float qwqy = Q.w * Q.y;
            float qwqz = Q.w * Q.z;
            float qxqy = Q.x * Q.y;
            float qxqz = Q.x * Q.z;
            float qyqz = Q.y * Q.z;
            FusionVector accelerometer = new FusionVector()
            {
                axis = new FusionVectorAxis()
                {
                    x = 2.0f * ((qwqw - 0.5f + Q.x * Q.x) * A.x + (qxqy - qwqz) * A.y + (qxqz + qwqy) * A.z),
                    y = 2.0f * ((qxqy + qwqz) * A.x + (qwqw - 0.5f + Q.y * Q.y) * A.y + (qyqz - qwqx) * A.z),
                    z = 2.0f * ((qxqz - qwqy) * A.x + (qyqz + qwqx) * A.y + (qwqw - 0.5f + Q.z * Q.z) * A.z),
                }
            }; // rotation matrix multiplied with the accelerometer

            // Remove gravity from accelerometer measurement
            switch (this.settings.convention)
            {
                case FusionConvention.FusionConventionNwu:
                case FusionConvention.FusionConventionEnu:
                    accelerometer.axis.z -= 1.0f;
                    break;
                case FusionConvention.FusionConventionNed:
                    accelerometer.axis.z += 1.0f;
                    break;
            }
            return accelerometer;
        }

        /**
         * @brief Returns the this algorithm internal states.
         * @param this this algorithm structure.
         * @return this algorithm internal states.
         */
        public FusionAhrsInternalStates FusionAhrsGetInternalStates()
        {
            FusionAhrsInternalStates internalStates = new FusionAhrsInternalStates()
            {
                accelerationError = FusionRadiansToDegrees(FusionAsin(2.0f * FusionVectorMagnitude(this.halfAccelerometerFeedback))),
                accelerometerIgnored = this.accelerometerIgnored,
                accelerationRecoveryTrigger = this.settings.recoveryTriggerPeriod == 0 ? 0.0f : (float)this.accelerationRecoveryTrigger / (float)this.settings.recoveryTriggerPeriod,
                magneticError = FusionRadiansToDegrees(FusionAsin(2.0f * FusionVectorMagnitude(this.halfMagnetometerFeedback))),
                magnetometerIgnored = this.magnetometerIgnored,
                magneticRecoveryTrigger = this.settings.recoveryTriggerPeriod == 0 ? 0.0f : (float)this.magneticRecoveryTrigger / (float)this.settings.recoveryTriggerPeriod,
            };
            return internalStates;
        }

        /**
         * @brief Returns the this algorithm flags.
         * @param this this algorithm structure.
         * @return this algorithm flags.
         */
        public FusionAhrsFlags FusionAhrsGetFlags()
        {
            FusionAhrsFlags flags = new FusionAhrsFlags()
            {
                initialising = this.initialising,
                angularRateRecovery = this.angularRateRecovery,
                accelerationRecovery = this.accelerationRecoveryTrigger > this.accelerationRecoveryTimeout,
                magneticRecovery = this.magneticRecoveryTrigger > this.magneticRecoveryTimeout,
            };
            return flags;
        }

        /**
         * @brief Sets the heading of the orientation measurement provided by the this
         * algorithm.  This function can be used to reset drift in heading when the this
         * algorithm is being used without a magnetometer.
         * @param this this algorithm structure.
         * @param heading Heading angle in degrees.
         */
        public void FusionAhrsSetHeading(float heading)
        {
            FusionQuaternionElement Q = this.quaternion.element;
            float yaw = (float)Math.Atan2(Q.w * Q.z + Q.x * Q.y, 0.5f - Q.y * Q.y - Q.z * Q.z);
            float halfYawMinusHeading = 0.5f * (yaw - FusionDegreesToRadians(heading));
            FusionQuaternion rotation = new FusionQuaternion()
            {
                element = new FusionQuaternionElement()
                {
                    w = (float)Math.Cos(halfYawMinusHeading),
                    x = 0.0f,
                    y = 0.0f,
                    z = -1.0f * (float)Math.Sin(halfYawMinusHeading),
                }
            };
            this.quaternion = FusionQuaternionMultiply(rotation, this.quaternion);
        }

        //------------------------------------------------------------------------------
        // End of file

    }
}
