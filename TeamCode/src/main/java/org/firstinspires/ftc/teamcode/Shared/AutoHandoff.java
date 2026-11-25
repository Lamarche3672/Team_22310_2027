package org.firstinspires.ftc.teamcode.Shared;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Carries "which way is forward for the drivers" from autonomous into teleop.
 *
 * Teleop re-zeroes the Pinpoint at INIT, so on its own it only knows "forward = whichever way
 * the robot happens to face". After an auto that is rarely away from the drivers, and driver 2
 * used to have to turn the robot and tap START before field-centric made sense.
 *
 * Autonomous knows the robot's real heading on the field when it ends. It leaves it here, and
 * teleop uses it to set forward by itself. Static on purpose: the two OpModes are separate
 * objects, and this is the only thing that outlives both.
 *
 * Field coordinates, as used by our Pedro routes: x runs left to right as seen from the
 * audience, y runs away from the audience, headings are counter-clockwise from +x. The red
 * drivers stand on the left (manual section 9.5), so for them "away from me" is +x.
 */
public final class AutoHandoff {
    /** The field heading that points away from the RED drivers. Blue is the opposite. */
    public static final double RED_DRIVERS_FACE_DEGREES = 0;
    /**
     * How long an auto's heading is trusted. In a match teleop is initialised within the 8
     * second changeover. In practice the robot is usually picked up and put somewhere else
     * before teleop, and by then this has run out, so a stale heading is not used.
     */
    private static final long FRESH_NANOS = 30_000_000_000L;

    private static double endHeading = Double.NaN;
    private static boolean endedBlue;
    private static long endNanos;

    private AutoHandoff() { }

    /** Called by autonomous as it ends, with the robot's field heading in radians. */
    public static void save(double fieldHeadingRadians, boolean blueAlliance) {
        if (!Double.isFinite(fieldHeadingRadians)) return;
        endHeading = fieldHeadingRadians;
        endedBlue = blueAlliance;
        endNanos = System.nanoTime();
    }

    /**
     * For teleop, right after it has zeroed the Pinpoint with the robot standing where auto
     * left it: the Pinpoint heading that means "facing away from the drivers". NaN if there
     * was no recent auto. One use only, so a later re-INIT somewhere else does not reuse it.
     */
    public static double takeForwardOffset() {
        double offset = Double.NaN;
        if (Double.isFinite(endHeading) && System.nanoTime() - endNanos <= FRESH_NANOS) {
            double driversFace = Math.toRadians(RED_DRIVERS_FACE_DEGREES) + (endedBlue ? Math.PI : 0);
            // The freshly zeroed Pinpoint reads 0 where the field heading was endHeading.
            offset = AngleUnit.normalizeRadians(driversFace - endHeading);
        }
        endHeading = Double.NaN;
        return offset;
    }
}
