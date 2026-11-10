package org.firstinspires.ftc.teamcode.pedro.procedures;

import java.util.List;

/** Reject incomplete or non-finite tuning results before they are used or saved. */
final class ForesightMeasurements {
    private ForesightMeasurements() { }

    static double positive(String stage, double value) {
        if (!Double.isFinite(value) || value <= 0.0) throw invalid(stage);
        return value;
    }

    static List<Double> finite(String stage, List<Double> values, int count) {
        if (values == null || values.size() != count) throw invalid(stage);
        for (Double value : values) {
            if (value == null || !Double.isFinite(value)) throw invalid(stage);
        }
        return values;
    }

    private static IllegalStateException invalid(String stage) {
        return new IllegalStateException(stage + " produced an invalid or incomplete result. "
                + "Check localization and retry Foresight tuning.");
    }
}
