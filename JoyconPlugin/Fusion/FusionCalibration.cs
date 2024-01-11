using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using static JoyconPlugin.Fusion.FusionMath;

namespace JoyconPlugin.Fusion
{
    public class FusionCalibration
    {
        //------------------------------------------------------------------------------
        // Inline functions

        /**
         * @brief Gyroscope and accelerometer calibration model.
         * @param uncalibrated Uncalibrated measurement.
         * @param misalignment Misalignment matrix.
         * @param sensitivity Sensitivity.
         * @param offset Offset.
         * @return Calibrated measurement.
         */
        public static FusionVector FusionCalibrationInertial(FusionVector uncalibrated, FusionMatrix misalignment, FusionVector sensitivity, FusionVector offset)
        {
            return FusionMatrixMultiplyVector(misalignment, FusionVectorHadamardProduct(FusionVectorSubtract(uncalibrated, offset), sensitivity));
        }

        /**
         * @brief Magnetometer calibration model.
         * @param uncalibrated Uncalibrated measurement.
         * @param softIronMatrix Soft-iron matrix.
         * @param hardIronOffset Hard-iron offset.
         * @return Calibrated measurement.
         */
        public static FusionVector FusionCalibrationMagnetic(FusionVector uncalibrated, FusionMatrix softIronMatrix, FusionVector hardIronOffset)
        {
            return FusionMatrixMultiplyVector(softIronMatrix, FusionVectorSubtract(uncalibrated, hardIronOffset));
        }


        //------------------------------------------------------------------------------
        // End of file
    }
}
