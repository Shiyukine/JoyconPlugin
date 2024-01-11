using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using static JoyconPlugin.Fusion.FusionMath;

namespace JoyconPlugin.Fusion
{
    public class FusionOffset
    {
        float filterCoefficient;
        uint timeout;
        uint timer;
        FusionVector gyroscopethis;

        const float CUTOFF_FREQUENCY = 0.02f;
        const int TIMEOUT = 5;
        const float THRESHOLD = 3.0f;

        //------------------------------------------------------------------------------
        // Functions

        /**
         * @brief Initialises the gyroscope this algorithm.
         * @param this Gyroscope this algorithm structure.
         * @param sampleRate Sample rate in Hz.
         */
        public FusionOffset(uint sampleRate)
        {
            this.filterCoefficient = 2.0f * (float)M_PI * CUTOFF_FREQUENCY * (1.0f / (float)sampleRate);
            this.timeout = TIMEOUT * sampleRate;
            this.timer = 0;
            this.gyroscopethis = FUSION_VECTOR_ZERO;
        }

        /**
         * @brief Updates the gyroscope this algorithm and returns the corrected
         * gyroscope measurement.
         * @param this Gyroscope this algorithm structure.
         * @param gyroscope Gyroscope measurement in degrees per second.
         * @return Corrected gyroscope measurement in degrees per second.
         */
        public FusionVector FusionOffsetUpdate(FusionVector gyroscope)
        {

            // Subtract this from gyroscope measurement
            gyroscope = FusionVectorSubtract(gyroscope, this.gyroscopethis);

            // Reset timer if gyroscope not stationary
            if ((Math.Abs(gyroscope.axis.x) > THRESHOLD) || (Math.Abs(gyroscope.axis.y) > THRESHOLD) || (Math.Abs(gyroscope.axis.z) > THRESHOLD))
            {
                this.timer = 0;
                return gyroscope;
            }

            // Increment timer while gyroscope stationary
            if (this.timer < this.timeout)
            {
                this.timer++;
                return gyroscope;
            }

            // Adjust this if timer has elapsed
            this.gyroscopethis = FusionVectorAdd(this.gyroscopethis, FusionVectorMultiplyScalar(gyroscope, this.filterCoefficient));
            return gyroscope;
        }

        //------------------------------------------------------------------------------
        // End of file
    }
}
