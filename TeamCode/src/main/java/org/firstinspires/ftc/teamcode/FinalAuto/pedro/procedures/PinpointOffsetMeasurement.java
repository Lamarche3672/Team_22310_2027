package org.firstinspires.ftc.teamcode.FinalAuto.pedro.procedures;

final class PinpointOffsetMeasurement {
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

    static PinpointOffsetMeasurement solve(double x, double y, double turn) {
        if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(turn))
            return failed("The Pinpoint position was not a real number. Run the step again.");
        double degrees = Math.abs(Math.toDegrees(turn));
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
