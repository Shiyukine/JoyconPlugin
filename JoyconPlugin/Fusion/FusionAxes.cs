using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using static JoyconPlugin.Fusion.FusionMath;

namespace JoyconPlugin.Fusion
{
    public class FusionAxes
    {
        //------------------------------------------------------------------------------
        // Definitions

        /**
         * @brief Axes alignment describing the sensor axes relative to the body axes.
         * For example, if the body X axis is aligned with the sensor Y axis and the
         * body Y axis is aligned with sensor X axis but pointing the opposite direction
         * then alignment is +Y-X+Z.
         */
        enum FusionAxesAlignment
        {
            FusionAxesAlignmentPXPYPZ, /* +X+Y+Z */
            FusionAxesAlignmentPXNZPY, /* +X-Z+Y */
            FusionAxesAlignmentPXNYNZ, /* +X-Y-Z */
            FusionAxesAlignmentPXPZNY, /* +X+Z-Y */
            FusionAxesAlignmentNXPYNZ, /* -X+Y-Z */
            FusionAxesAlignmentNXPZPY, /* -X+Z+Y */
            FusionAxesAlignmentNXNYPZ, /* -X-Y+Z */
            FusionAxesAlignmentNXNZNY, /* -X-Z-Y */
            FusionAxesAlignmentPYNXPZ, /* +Y-X+Z */
            FusionAxesAlignmentPYNZNX, /* +Y-Z-X */
            FusionAxesAlignmentPYPXNZ, /* +Y+X-Z */
            FusionAxesAlignmentPYPZPX, /* +Y+Z+X */
            FusionAxesAlignmentNYPXPZ, /* -Y+X+Z */
            FusionAxesAlignmentNYNZPX, /* -Y-Z+X */
            FusionAxesAlignmentNYNXNZ, /* -Y-X-Z */
            FusionAxesAlignmentNYPZNX, /* -Y+Z-X */
            FusionAxesAlignmentPZPYNX, /* +Z+Y-X */
            FusionAxesAlignmentPZPXPY, /* +Z+X+Y */
            FusionAxesAlignmentPZNYPX, /* +Z-Y+X */
            FusionAxesAlignmentPZNXNY, /* +Z-X-Y */
            FusionAxesAlignmentNZPYPX, /* -Z+Y+X */
            FusionAxesAlignmentNZNXPY, /* -Z-X+Y */
            FusionAxesAlignmentNZNYNX, /* -Z-Y-X */
            FusionAxesAlignmentNZPXNY, /* -Z+X-Y */
        }

//------------------------------------------------------------------------------
// Inline functions

/**
 * @brief Swaps sensor axes for alignment with the body axes.
 * @param sensor Sensor axes.
 * @param alignment Axes alignment.
 * @return Sensor axes aligned with the body axes.
 */
static FusionVector FusionAxesSwap(FusionVector sensor, FusionAxesAlignment alignment) {
    FusionVector result = new FusionVector();
    switch (alignment) {
        case FusionAxesAlignment.FusionAxesAlignmentPXPYPZ:
            break;
        case FusionAxesAlignment.FusionAxesAlignmentPXNZPY:
            result.axis.x = +sensor.axis.x;
            result.axis.y = -sensor.axis.z;
            result.axis.z = +sensor.axis.y;
            return result;
        case FusionAxesAlignment.FusionAxesAlignmentPXNYNZ:
            result.axis.x = +sensor.axis.x;
            result.axis.y = -sensor.axis.y;
            result.axis.z = -sensor.axis.z;
            return result;
        case FusionAxesAlignment.FusionAxesAlignmentPXPZNY:
            result.axis.x = +sensor.axis.x;
            result.axis.y = +sensor.axis.z;
            result.axis.z = -sensor.axis.y;
            return result;
        case FusionAxesAlignment.FusionAxesAlignmentNXPYNZ:
            result.axis.x = -sensor.axis.x;
            result.axis.y = +sensor.axis.y;
            result.axis.z = -sensor.axis.z;
            return result;
        case FusionAxesAlignment.FusionAxesAlignmentNXPZPY:
            result.axis.x = -sensor.axis.x;
            result.axis.y = +sensor.axis.z;
            result.axis.z = +sensor.axis.y;
            return result;
        case FusionAxesAlignment.FusionAxesAlignmentNXNYPZ:
            result.axis.x = -sensor.axis.x;
            result.axis.y = -sensor.axis.y;
            result.axis.z = +sensor.axis.z;
            return result;
        case FusionAxesAlignment.FusionAxesAlignmentNXNZNY:
            result.axis.x = -sensor.axis.x;
            result.axis.y = -sensor.axis.z;
            result.axis.z = -sensor.axis.y;
            return result;
        case FusionAxesAlignment.FusionAxesAlignmentPYNXPZ:
            result.axis.x = +sensor.axis.y;
            result.axis.y = -sensor.axis.x;
            result.axis.z = +sensor.axis.z;
            return result;
        case FusionAxesAlignment.FusionAxesAlignmentPYNZNX:
            result.axis.x = +sensor.axis.y;
            result.axis.y = -sensor.axis.z;
            result.axis.z = -sensor.axis.x;
            return result;
        case FusionAxesAlignment.FusionAxesAlignmentPYPXNZ:
            result.axis.x = +sensor.axis.y;
            result.axis.y = +sensor.axis.x;
            result.axis.z = -sensor.axis.z;
            return result;
        case FusionAxesAlignment.FusionAxesAlignmentPYPZPX:
            result.axis.x = +sensor.axis.y;
            result.axis.y = +sensor.axis.z;
            result.axis.z = +sensor.axis.x;
            return result;
        case FusionAxesAlignment.FusionAxesAlignmentNYPXPZ:
            result.axis.x = -sensor.axis.y;
            result.axis.y = +sensor.axis.x;
            result.axis.z = +sensor.axis.z;
            return result;
        case FusionAxesAlignment.FusionAxesAlignmentNYNZPX:
            result.axis.x = -sensor.axis.y;
            result.axis.y = -sensor.axis.z;
            result.axis.z = +sensor.axis.x;
            return result;
        case FusionAxesAlignment.FusionAxesAlignmentNYNXNZ:
            result.axis.x = -sensor.axis.y;
            result.axis.y = -sensor.axis.x;
            result.axis.z = -sensor.axis.z;
            return result;
        case FusionAxesAlignment.FusionAxesAlignmentNYPZNX:
            result.axis.x = -sensor.axis.y;
            result.axis.y = +sensor.axis.z;
            result.axis.z = -sensor.axis.x;
            return result;
        case FusionAxesAlignment.FusionAxesAlignmentPZPYNX:
            result.axis.x = +sensor.axis.z;
            result.axis.y = +sensor.axis.y;
            result.axis.z = -sensor.axis.x;
            return result;
        case FusionAxesAlignment.FusionAxesAlignmentPZPXPY:
            result.axis.x = +sensor.axis.z;
            result.axis.y = +sensor.axis.x;
            result.axis.z = +sensor.axis.y;
            return result;
        case FusionAxesAlignment.FusionAxesAlignmentPZNYPX:
            result.axis.x = +sensor.axis.z;
            result.axis.y = -sensor.axis.y;
            result.axis.z = +sensor.axis.x;
            return result;
        case FusionAxesAlignment.FusionAxesAlignmentPZNXNY:
            result.axis.x = +sensor.axis.z;
            result.axis.y = -sensor.axis.x;
            result.axis.z = -sensor.axis.y;
            return result;
        case FusionAxesAlignment.FusionAxesAlignmentNZPYPX:
            result.axis.x = -sensor.axis.z;
            result.axis.y = +sensor.axis.y;
            result.axis.z = +sensor.axis.x;
            return result;
        case FusionAxesAlignment.FusionAxesAlignmentNZNXPY:
            result.axis.x = -sensor.axis.z;
            result.axis.y = -sensor.axis.x;
            result.axis.z = +sensor.axis.y;
            return result;
        case FusionAxesAlignment.FusionAxesAlignmentNZNYNX:
            result.axis.x = -sensor.axis.z;
            result.axis.y = -sensor.axis.y;
            result.axis.z = -sensor.axis.x;
            return result;
        case FusionAxesAlignment.FusionAxesAlignmentNZPXNY:
            result.axis.x = -sensor.axis.z;
            result.axis.y = +sensor.axis.x;
            result.axis.z = -sensor.axis.y;
            return result;
    }
    return sensor; // avoid compiler warning
}

//------------------------------------------------------------------------------
// End of file
    }
}
