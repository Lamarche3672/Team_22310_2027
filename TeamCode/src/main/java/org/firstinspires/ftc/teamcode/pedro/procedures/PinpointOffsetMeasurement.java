package org.firstinspires.ftc.teamcode.pedro.procedures;

/**
 * Works out where the two pods sit on the robot from one spin on the spot.
 *
 * <p>The tuner runs the Pinpoint with both offsets set to zero. When the robot then spins
 * about its center, the Pinpoint wrongly thinks it also slid sideways. For a spin of
 * {@code turn} radians (CCW positive), with the forward pod {@code a} inches to the left
 * of center and the sideways pod {@code b} inches ahead of center, it reports:
 * <pre>
 *   x = -a sin(turn) + b (cos(turn) - 1)
 *   y =  a (cos(turn) - 1) + b sin(turn)
 * </pre>
 * Solving those two equations gives a and b for any turn angle. At exactly half a turn
 * this is Pedro's own formula: a = -y / 2, b = -x / 2.
 */
final class PinpointOffsetMeasurement {
    /** The spin must be near half a turn; small angles make the answer too sensitive. */
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

    /**
     * @param x    reported X after the spin, in the frame the robot started in (inches)
     * @param y    reported Y after the spin, same frame (inches)
     * @param turn total rotation in radians, CCW positive; may be more than half a turn
     */
    static PinpointOffsetMeasurement solve(double x, double y, double turn) {
        if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(turn))
            return failed("The Pinpoint position was not a real number. Run the step again.");
        double degrees = Math.abs(Math.toDegrees(turn));
        // The tiny margin keeps exactly 150 or 210 from being rejected by rounding.
        if (degrees < MIN_TURN_DEGREES - 1e-6 || degrees > MAX_TURN_DEGREES + 1e-6)
            return failed(String.format("You turned %.0f degrees. Turn the robot about half a turn"
                    + " (%.0f to %.0f degrees) on the spot, then press Stop.",
                    degrees, MIN_TURN_DEGREES, MAX_TURN_DEGREES));
        double sin = Math.sin(turn);
        double oneMinusCos = 1.0 - Math.cos(turn);
        double a = -(sin * x + oneMinusCos * y) / (2.0 * oneMinusCos);
        double b = -(oneMinusCos * x - sin * y) / (2.0 * oneMinusCos);
        return new PinpointOffsetMeasurement(a, b, null);
    }

    /** Null when the measurement is good; otherwise what to do differently. */
    String failure() {
        return failure;
    }

    /** How far LEFT of the robot's center the forward pod is, inches (right is negative). */
    double xPodOffset() {
        return xPodOffset;
    }

    /** How far AHEAD of the robot's center the sideways pod is, inches (behind is negative). */
    double yPodOffset() {
        return yPodOffset;
    }
}
