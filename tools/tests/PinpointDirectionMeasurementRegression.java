package org.firstinspires.ftc.teamcode.pedro.procedures;

/** Standalone regression tests; no FTC hardware or test-library dependency needed. */
public final class PinpointDirectionMeasurementRegression {
    private static int checks;

    private static void check(boolean condition, String message) {
        checks++;
        if (!condition) throw new AssertionError(message);
    }

    private static PinpointDirectionMeasurement measurement(boolean forward, int x, int y) {
        PinpointDirectionMeasurement m = new PinpointDirectionMeasurement(forward, 500.0);
        m.sample(x, y);
        return m;
    }

    public static void main(String[] args) {
        PinpointDirectionMeasurement m = measurement(true, 3000, 0);
        check(m.failure() == null && !m.reversed(), "forward positive");
        check(m.peakInches() == 6.0, "ticks converted to inches");
        m.sample(0, 0);
        check(m.failure() == null && !m.reversed(), "return to start retains direction");
        m = measurement(true, -3000, 0);
        check(m.failure() == null && m.reversed(), "forward negative");
        m.sample(0, 0);
        check(m.failure() == null && m.reversed(), "reversed return to start");
        m = measurement(false, 0, 3000);
        check(m.failure() == null && !m.reversed(), "strafe positive");
        m = measurement(false, 0, -3000);
        check(m.failure() == null && m.reversed(), "strafe negative");
        check(measurement(true, 0, 0).failure() != null, "no motion rejected");
        check(!measurement(true, 0, 0).failure().contains("Press Start"), "automatic start instructions");
        check(measurement(true, 499, 0).failure() != null, "sub-inch noise rejected");
        check(measurement(true, 500, 0).failure() == null, "one-inch threshold preserved");
        check(measurement(true, 0, 3000).failure().contains("swapped"), "wrong X axis diagnosed");
        check(measurement(false, 3000, 0).failure().contains("swapped"), "wrong Y axis diagnosed");
        check(measurement(true, 1000, 3000).failure() != null, "other-axis dominance rejected");
        check(measurement(false, 3000, 1000).failure() != null, "strafe cross-axis dominance rejected");
        m = measurement(true, 3000, 100);
        check(m.failure() == null, "small cross-axis movement allowed");
        m.sample(-3000, 0);
        check(m.failure() != null, "ambiguous opposite pushes rejected");
        m = measurement(true, 3000, 0);
        m.fail("Pinpoint fault");
        check("Pinpoint fault".equals(m.failure()), "fault overrides good movement");
        try {
            m.reversed();
            throw new AssertionError("fault must not yield a direction");
        } catch (IllegalStateException expected) {
            checks++;
        }
        for (double invalid : new double[] {0.0, -1.0, Double.NaN, Double.POSITIVE_INFINITY}) {
            try {
                new PinpointDirectionMeasurement(true, invalid);
                throw new AssertionError("invalid resolution accepted");
            } catch (IllegalArgumentException expected) {
                checks++;
            }
        }
        int start = Integer.MAX_VALUE - 10;
        int end = start + 3000;
        check(measurement(true, end - start, 0).failure() == null, "int rollover delta preserved");
        System.out.println("PASS: " + checks + " Pinpoint direction regression checks");
    }
}
