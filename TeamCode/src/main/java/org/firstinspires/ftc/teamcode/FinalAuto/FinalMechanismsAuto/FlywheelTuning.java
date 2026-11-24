package org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto;

import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.Locale;

public final class FlywheelTuning {
    private static final String FILE_NAME = "flywheel_gains.txt";
    private static final double LARGEST_SENSIBLE_VALUE = 1000;

    private FlywheelTuning() { }

    public static double[] load() {
        try {
            File file = AppUtil.getInstance().getSettingsFile(FILE_NAME);
            if (!file.exists()) return null;
            String[] parts = ReadWriteFile.readFile(file).trim().split("\\s+");
            if (parts.length < 2) return null;
            double p = Double.parseDouble(parts[0]);
            double f = Double.parseDouble(parts[1]);
            double i = parts.length >= 3 ? Double.parseDouble(parts[2]) : 0;
            return isUsable(p, f, i) ? new double[] {p, f, i} : null;
        } catch (Throwable problem) {
            return null;
        }
    }

    public static boolean save(double p, double f, double i) {
        if (!isUsable(p, f, i)) return false;
        try {
            File file = AppUtil.getInstance().getSettingsFile(FILE_NAME);
            ReadWriteFile.writeFile(file, String.format(Locale.US, "%.6f %.6f %.6f", p, f, i));
            double[] readBack = load();
            return readBack != null && Math.abs(readBack[0] - p) < 1e-4 && Math.abs(readBack[1] - f) < 1e-4;
        } catch (Throwable problem) {
            return false;
        }
    }

    static boolean isUsable(double p, double f, double i) {
        return Double.isFinite(p) && Double.isFinite(f) && Double.isFinite(i)
                && p >= 0 && f >= 0 && i >= 0 && (p > 0 || f > 0)
                && p <= LARGEST_SENSIBLE_VALUE && f <= LARGEST_SENSIBLE_VALUE && i <= LARGEST_SENSIBLE_VALUE;
    }
}
