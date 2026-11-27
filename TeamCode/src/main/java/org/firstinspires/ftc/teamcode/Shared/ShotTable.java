package org.firstinspires.ftc.teamcode.Shared;

import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * The list of "from this far away, spin the flywheels this fast" that the drive team saves
 * during shot setup (gamepad 1 BACK, then A).
 *
 * Each saved shot is a pair {distance in inches, flywheel speed in ticks per second}. The list
 * is kept sorted by distance. Asking for a distance between two saved shots gives a speed in
 * between; closer than the closest or further than the furthest gives that end's speed.
 *
 * It is stored as a small text file on the Control Hub (FIRST/settings/shot_speeds_v2.txt),
 * one "distance speed" per line, so it survives power cycles and new code installs.
 *
 * Why "_v2": the first file ended up holding speeds that were far too low, and because a
 * saved shot beats the Y preset whenever the camera sees the hive, every shot with the
 * Limelight plugged in fell short (unplug it and the same shot went in 8 times out of 8).
 * A new file name means that old list is simply never read again. To start over another
 * time, bump the number.
 */
public final class ShotTable {
    private static final String FILE_NAME = "shot_speeds_v2.txt";
    /** Past the furthest saved shot, the speed climbs to the far speed over this many inches. */
    public static final double FAR_RAMP_INCHES = 36;
    // "_v2" for the same reason as the shot list: for one day the A button in shot setup
    // overwrote the HIGH preset whenever the camera had no distance, so a speed meant for a
    // close shot could end up as the speed for everything. That file is never read again.
    private static final String PRESET_FILE_NAME = "shot_presets_v2.txt";
    /**
     * The Y presets before anybody has saved their own. One place, so teleop and autonomous
     * start from the same number. HIGH is the back-wall shot.
     */
    public static final double DEFAULT_HIGH_PRESET = 2300;
    public static final double DEFAULT_LOW_PRESET = 1500;
    /** Saving within this distance of an existing shot replaces it instead of adding another. */
    public static final double SAME_SPOT_INCHES = 6;
    public static final double DELETE_WITHIN_INCHES = 12;
    public static final int MAX_POINTS = 12;

    private ShotTable() { }

    /** The speed for this distance, or NaN if nothing is saved or the distance is unknown. */
    public static double speedFor(List<double[]> points, double distance) {
        if (points.isEmpty() || !Double.isFinite(distance)) return Double.NaN;
        double[] first = points.get(0);
        if (distance <= first[0]) return first[1];
        for (int i = 1; i < points.size(); i++) {
            double[] near = points.get(i - 1);
            double[] far = points.get(i);
            if (distance <= far[0]) {
                double part = (distance - near[0]) / (far[0] - near[0]);
                return near[1] + (far[1] - near[1]) * part;
            }
        }
        return points.get(points.size() - 1)[1];
    }

    /**
     * The speed to use, given a "far speed" (the HIGH preset in teleop, the auto's own speed
     * in autonomous). Inside the saved range it is the same as speedFor(points, distance).
     * Further out than the furthest saved shot it climbs toward farSpeed instead of staying
     * put: a shot saved from close up must never make a long shot weak. It never goes below
     * the furthest saved speed, so a far shot that was saved on purpose is left alone.
     */
    public static double speedFor(List<double[]> points, double distance, double farSpeed) {
        double speed = speedFor(points, distance);
        if (!Double.isFinite(speed) || !Double.isFinite(farSpeed)) return speed;
        double[] furthest = points.get(points.size() - 1);
        if (distance <= furthest[0] || farSpeed <= furthest[1]) return speed;
        double part = Math.min(1.0, (distance - furthest[0]) / FAR_RAMP_INCHES);
        return furthest[1] + (farSpeed - furthest[1]) * part;
    }

    /** Adds a shot, keeping the list sorted. Returns false if it was nonsense or the list is full. */
    public static boolean add(List<double[]> points, double distance, double speed) {
        if (!Double.isFinite(distance) || !Double.isFinite(speed) || distance <= 0 || speed <= 0) return false;
        int nearest = nearest(points, distance);
        if (nearest >= 0 && Math.abs(points.get(nearest)[0] - distance) <= SAME_SPOT_INCHES) {
            points.remove(nearest);
        } else if (points.size() >= MAX_POINTS) {
            return false;
        }
        int at = 0;
        while (at < points.size() && points.get(at)[0] < distance) at++;
        points.add(at, new double[] {distance, speed});
        return true;
    }

    public static boolean removeNear(List<double[]> points, double distance) {
        int nearest = nearest(points, distance);
        if (nearest < 0 || Math.abs(points.get(nearest)[0] - distance) > DELETE_WITHIN_INCHES) return false;
        points.remove(nearest);
        return true;
    }

    private static int nearest(List<double[]> points, double distance) {
        if (!Double.isFinite(distance)) return -1;
        int best = -1;
        for (int i = 0; i < points.size(); i++) {
            if (best < 0 || Math.abs(points.get(i)[0] - distance) < Math.abs(points.get(best)[0] - distance)) best = i;
        }
        return best;
    }

    public static String describe(List<double[]> points) {
        if (points.isEmpty()) return "none yet";
        StringBuilder text = new StringBuilder();
        for (double[] point : points) {
            if (text.length() > 0) text.append(", ");
            text.append(String.format(Locale.US, "%.0f in = %.0f", point[0], point[1]));
        }
        return text.toString();
    }

    public static List<double[]> parse(String text) {
        List<double[]> points = new ArrayList<>();
        if (text == null) return points;
        for (String line : text.split("\n")) {
            String[] parts = line.trim().split("\\s+");
            if (parts.length < 2) continue;
            try {
                add(points, Double.parseDouble(parts[0]), Double.parseDouble(parts[1]));
            } catch (NumberFormatException skip) {
            }
        }
        return points;
    }

    public static String format(List<double[]> points) {
        StringBuilder text = new StringBuilder();
        for (double[] point : points) text.append(String.format(Locale.US, "%.1f %.0f\n", point[0], point[1]));
        return text.toString();
    }

    // Reading and writing must never be able to stop an OpMode, so every kind of failure
    // (no file, damaged file, no storage permission, not running on a robot) is swallowed.
    public static List<double[]> load() {
        try {
            File file = AppUtil.getInstance().getSettingsFile(FILE_NAME);
            if (!file.exists()) return new ArrayList<>();
            return parse(ReadWriteFile.readFile(file));
        } catch (Throwable problem) {
            return new ArrayList<>();
        }
    }

    /**
     * The two Y presets {high, low} as last saved from shot setup, or null if none were ever
     * saved. They live in their own small file next to the shot list, for the same reason:
     * a speed found on the practice field should still be there after a restart.
     */
    public static double[] loadPresets() {
        try {
            File file = AppUtil.getInstance().getSettingsFile(PRESET_FILE_NAME);
            if (!file.exists()) return null;
            String[] parts = ReadWriteFile.readFile(file).trim().split("\\s+");
            double high = Double.parseDouble(parts[0]);
            double low = Double.parseDouble(parts[1]);
            return Double.isFinite(high) && Double.isFinite(low) && high > 0 && low > 0 ? new double[] {high, low} : null;
        } catch (Throwable problem) {
            return null;
        }
    }

    public static boolean savePresets(double high, double low) {
        try {
            File file = AppUtil.getInstance().getSettingsFile(PRESET_FILE_NAME);
            ReadWriteFile.writeFile(file, String.format(Locale.US, "%.0f %.0f\n", high, low));
            double[] back = loadPresets();
            return back != null && Math.abs(back[0] - high) < 1 && Math.abs(back[1] - low) < 1;
        } catch (Throwable problem) {
            return false;
        }
    }

    public static boolean save(List<double[]> points) {
        try {
            File file = AppUtil.getInstance().getSettingsFile(FILE_NAME);
            ReadWriteFile.writeFile(file, format(points));
            // Read it straight back: the only way to know the write really worked.
            return load().size() == points.size();
        } catch (Throwable problem) {
            return false;
        }
    }
}
