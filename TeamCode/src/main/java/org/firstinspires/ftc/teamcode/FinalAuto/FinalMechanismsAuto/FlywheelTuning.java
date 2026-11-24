package org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto;

import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.Locale;

/**
 * Remembers the flywheel P and F found with the teleop tuner.
 *
 * In MAIN TELEOP, press BACK to unlock tuning, tune with the d-pad and press A: the numbers are written to a small
 * file on the Control Hub (FIRST/settings/flywheel_tuning.txt). From then on teleop and
 * autonomous both start with them, also after the robot is switched off.
 *
 * Numbers typed into FlywheelAuto.P and FlywheelAuto.F always win over the saved file.
 */
public final class FlywheelTuning {
    private static final String FILE_NAME = "flywheel_tuning.txt";
    /** Anything above this is not a real tuning value; the file is ignored. */
    private static final double LARGEST_SENSIBLE_VALUE = 1000;

    private FlywheelTuning() { }

    /** The saved {P, F, I}, or null when nothing usable is saved. Never throws. */
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
            // A missing, damaged or unreadable file must never stop an OpMode.
            return null;
        }
    }

    /** Saves P, F and I. Returns true when they were written and read back correctly. */
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
