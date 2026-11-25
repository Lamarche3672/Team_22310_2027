package org.firstinspires.ftc.teamcode.FinalAuto.pedro.procedures;

/**
 * The maths behind the Pinpoint Tuner's "spin the robot half a turn" step: it works out where
 * the two odometry pods sit relative to the centre the robot turns about.
 *
 * With the offsets set to zero, a robot that only spins on the spot still LOOKS like it moved,
 * because the pods swing round the centre on a circle. How far it seems to have moved (x, y)
 * for a known turn tells us the radius of that circle, which is the pod offset.
 *
 * Kept separate from the tuner so it can be tested on a laptop (tools/tests).
 */
final class PinpointOffsetMeasurement {
    // About half a turn gives the biggest apparent movement, so the least error. A small turn
    // divides by a number near zero; a full turn brings the pods back where they started.
    static final double MIN_TURN_DEGREES = 150.0;
    static final double MAX_TURN_DEGREES = 210.0;

    private final double xPodOffset;
    private final double yPodOffset;
    private final String failure;

    private PinpointOffsetMeasurement(double xPodOffset, double yPodOffset, String failure) {
        this.xPodOffset = xPodOffset;
        this.yPodOffset = yPodOffset;
        this.failure = failure;
    }

    static PinpointOffsetMeasurement failed(String message) {
        return new PinpointOffsetMeasurement(Double.NaN, Double.NaN, message);
    }

    /** x, y: how far the Pinpoint thinks it moved (inches). turn: how far it turned (radians). */
    static PinpointOffsetMeasurement solve(double x, double y, double turn) {
        if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(turn))
            return failed("The Pinpoint position was not a real number. Run the step again.");
        double degrees = Math.abs(Math.toDegrees(turn));
        if (degrees < MIN_TURN_DEGREES - 1e-6 || degrees > MAX_TURN_DEGREES + 1e-6)
            return failed(String.format("You turned %.0f degrees. Turn the robot about half a turn"
                    + " (%.0f to %.0f degrees) on the spot, then press Stop.",
                    degrees, MIN_TURN_DEGREES, MAX_TURN_DEGREES));
        // A pod at (a, b) that is rotated by `turn` ends up at R(turn)(a, b). The Pinpoint
        // reports the difference, (x, y) = (R - I)(a, b), with its own sign convention.
        // Solving that 2x2 system for (a, b) gives the two lines below.
        double sin = Math.sin(turn);
        double oneMinusCos = 1.0 - Math.cos(turn);
        double a = -(sin * x + oneMinusCos * y) / (2.0 * oneMinusCos);
        double b = -(oneMinusCos * x - sin * y) / (2.0 * oneMinusCos);
        return new PinpointOffsetMeasurement(a, b, null);
    }

    String failure() {
        return failure;
    }

    double xPodOffset() {
        return xPodOffset;
    }

    double yPodOffset() {
        return yPodOffset;
    }
}
