package org.firstinspires.ftc.teamcode.pedro.procedures;

/** Standalone regression tests; no FTC hardware or test-library dependency needed. */
public final class PinpointOffsetMeasurementRegression {
    private static int checks;

    private static void check(boolean condition, String message) {
        checks++;
        if (!condition) throw new AssertionError(message);
    }

    private static boolean near(double actual, double expected) {
        return Math.abs(actual - expected) < 1e-3;
    }

    /**
     * What a Pinpoint with zero offsets reports after the robot spins about its center.
     * Stepped in tiny slices, the way the real device adds up pod movement:
     * the forward pod (a inches left of center) rolls backward, the sideways pod
     * (b inches ahead of center) rolls left.
     */
    private static double[] spin(double a, double b, double turn) {
        int steps = 200_000;
        double dTheta = turn / steps;
        double x = 0, y = 0;
        for (int i = 0; i < steps; i++) {
            double heading = (i + 0.5) * dTheta;
            double forward = -a * dTheta;
            double left = b * dTheta;
            x += forward * Math.cos(heading) - left * Math.sin(heading);
            y += forward * Math.sin(heading) + left * Math.cos(heading);
        }
        return new double[] {x, y};
    }

    public static void main(String[] args) {
        double[][] pods = {{3.5, -6.25}, {-4.0, 2.0}, {0.0, 5.0}, {6.0, 0.0}, {-2.5, -7.0}};
        double[] turnsDegrees = {180, 150, 165, 195, 210, -180, -160, -200};
        for (double[] pod : pods) {
            for (double degrees : turnsDegrees) {
                double turn = Math.toRadians(degrees);
                double[] reported = spin(pod[0], pod[1], turn);
                PinpointOffsetMeasurement m = PinpointOffsetMeasurement.solve(reported[0], reported[1], turn);
                String label = "pods " + pod[0] + ", " + pod[1] + " at " + degrees + " degrees";
                check(m.failure() == null, "accepted: " + label);
                check(near(m.xPodOffset(), pod[0]), "x offset: " + label + " got " + m.xPodOffset());
                check(near(m.yPodOffset(), pod[1]), "y offset: " + label + " got " + m.yPodOffset());
            }
        }

        // At exactly half a turn this must equal Pedro's own formula: -y / 2 and -x / 2.
        PinpointOffsetMeasurement half = PinpointOffsetMeasurement.solve(7.0, -4.0, Math.PI);
        check(near(half.xPodOffset(), 2.0) && near(half.yPodOffset(), -3.5), "matches Pedro at 180 degrees");

        for (double degrees : new double[] {0, 45, 90, 149, 211, 360, -90}) {
            PinpointOffsetMeasurement m = PinpointOffsetMeasurement.solve(1.0, 1.0, Math.toRadians(degrees));
            check(m.failure() != null, degrees + " degrees rejected");
            check(m.failure().contains("half a turn"), "rejection says what to do");
        }
        for (double bad : new double[] {Double.NaN, Double.POSITIVE_INFINITY}) {
            check(PinpointOffsetMeasurement.solve(bad, 0, Math.PI).failure() != null, "bad x rejected");
            check(PinpointOffsetMeasurement.solve(0, bad, Math.PI).failure() != null, "bad y rejected");
            check(PinpointOffsetMeasurement.solve(0, 0, bad).failure() != null, "bad turn rejected");
        }
        System.out.println("PASS: " + checks + " Pinpoint offset regression checks");
    }
}
