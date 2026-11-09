package org.firstinspires.ftc.teamcode.pedro.procedures;

/** Raw-tick direction evidence, independent of the Pinpoint's fused field pose. */
final class PinpointDirectionMeasurement {
    private final boolean forward;
    private final double ticksPerInch;
    private double positiveTicks;
    private double negativeTicks;
    private double otherTicks;
    private String fault;

    PinpointDirectionMeasurement(boolean forward, double ticksPerInch) {
        if (!Double.isFinite(ticksPerInch) || ticksPerInch <= 0.0)
            throw new IllegalArgumentException("Encoder resolution must be finite and positive.");
        this.forward = forward;
        this.ticksPerInch = ticksPerInch;
    }

    void sample(int xTicks, int yTicks) {
        double primary = forward ? xTicks : yTicks;
        double secondary = forward ? yTicks : xTicks;
        positiveTicks = Math.max(positiveTicks, primary);
        negativeTicks = Math.max(negativeTicks, -primary);
        otherTicks = Math.max(otherTicks, Math.abs(secondary));
    }

    void fail(String message) {
        fault = message;
    }

    double peakInches() {
        return Math.max(positiveTicks, negativeTicks) / ticksPerInch;
    }

    String failure() {
        if (fault != null) return fault;
        String axis = forward ? "X (forward)" : "Y (sideways)";
        String other = forward ? "Y" : "X";
        String push = forward ? "FORWARD" : "LEFT";
        if (peakInches() < 1.0) {
            if (otherTicks / ticksPerInch >= 1.0)
                return other + " encoder moved, but " + axis + " did not reach one inch. "
                        + "Check for swapped X/Y pod cables or the wrong push direction.";
            return axis + " travel was less than one inch. Press Start BEFORE pushing 6-12 inches "
                    + push + ", then Stop. If ticks stay at zero, check wheel contact and the pod cable.";
        }
        if (positiveTicks >= ticksPerInch && negativeTicks >= ticksPerInch)
            return "Movement was detected on both sides of the starting point. Repeat with one straight "
                    + push + " push; do not cross behind the starting point.";
        if (otherTicks > Math.max(positiveTicks, negativeTicks))
            return other + " encoder moved more than " + axis + ". Push straight " + push
                    + " without turning; check for swapped X/Y pod cables.";
        return null;
    }

    boolean reversed() {
        if (failure() != null) throw new IllegalStateException(failure());
        return negativeTicks > positiveTicks;
    }
}
