package org.firstinspires.ftc.teamcode.pedro.procedures;

import java.util.Arrays;
import java.util.List;

/** Standalone tests for rejecting bad tuning results before later stages. */
public final class ForesightMeasurementsRegression {
    private static int checks;
    private static void rejects(Runnable action) {
        try {
            action.run();
            throw new AssertionError("Invalid measurement accepted");
        } catch (IllegalStateException expected) {
            checks++;
        }
    }

    public static void main(String[] args) {
        if (ForesightMeasurements.positive("test", 1.0) != 1.0) throw new AssertionError();
        checks++;
        for (double n : new double[]{0, -1, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
            rejects(() -> ForesightMeasurements.positive("test", n));
        if (ForesightMeasurements.finite("test", List.of(-1.0, 0.0), 2).size() != 2) throw new AssertionError();
        checks++;
        rejects(() -> ForesightMeasurements.finite("test", null, 2));
        rejects(() -> ForesightMeasurements.finite("test", List.of(1.0), 2));
        rejects(() -> ForesightMeasurements.finite("test", Arrays.asList(1.0, null), 2));
        rejects(() -> ForesightMeasurements.finite("test", List.of(1.0, Double.NaN), 2));
        rejects(() -> ForesightMeasurements.finite("test", List.of(1.0, Double.POSITIVE_INFINITY), 2));
        System.out.println("PASS: " + checks + " Foresight measurement checks");
    }
}
