using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using static JoyconPlugin.Fusion.FusionMath;

namespace JoyconPlugin.Fusion
{
    public class FusionCompass
    {
        //------------------------------------------------------------------------------
        // Functions

        /**
         * @brief Calculates the magnetic heading.
         * @param convention Earth axes convention.
         * @param accelerometer Accelerometer measurement in any calibrated units.
         * @param magnetometer Magnetometer measurement in any calibrated units.
         * @return Heading angle in degrees.
         */
        float FusionCompassCalculateHeading(FusionConvention convention, FusionVector accelerometer, FusionVector magnetometer) {
    switch (convention) {
        case FusionConvention.FusionConventionNwu: {
            FusionVector west = FusionVectorNormalise(FusionVectorCrossProduct(accelerometer, magnetometer));
        FusionVector north = FusionVectorNormalise(FusionVectorCrossProduct(west, accelerometer));
            return FusionRadiansToDegrees((float)Math.Atan2(west.axis.x, north.axis.x));
    }
        case FusionConvention.FusionConventionEnu: {
            FusionVector west = FusionVectorNormalise(FusionVectorCrossProduct(accelerometer, magnetometer));
    FusionVector north = FusionVectorNormalise(FusionVectorCrossProduct(west, accelerometer));
    FusionVector east = FusionVectorMultiplyScalar(west, -1.0f);
            return FusionRadiansToDegrees((float)Math.Atan2(north.axis.x, east.axis.x));
}
        case FusionConvention.FusionConventionNed:
    {
        FusionVector up = FusionVectorMultiplyScalar(accelerometer, -1.0f);
        FusionVector west = FusionVectorNormalise(FusionVectorCrossProduct(up, magnetometer));
        FusionVector north = FusionVectorNormalise(FusionVectorCrossProduct(west, up));
        return FusionRadiansToDegrees((float)Math.Atan2(west.axis.x, north.axis.x));
    }
}
return 0; // avoid compiler warning
}

//------------------------------------------------------------------------------
// End of file
    }
}
