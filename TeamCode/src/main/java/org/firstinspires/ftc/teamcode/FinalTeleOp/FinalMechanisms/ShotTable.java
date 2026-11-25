package org.firstinspires.ftc.teamcode.FinalTeleOp.FinalMechanisms;

import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public final class ShotTable {
    private static final String FILE_NAME = "shot_speeds.txt";
    public static final double SAME_SPOT_INCHES = 6;
    public static final double DELETE_WITHIN_INCHES = 12;
    public static final int MAX_POINTS = 12;

    private ShotTable() { }

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

    public static List<double[]> load() {
        try {
            File file = AppUtil.getInstance().getSettingsFile(FILE_NAME);
            if (!file.exists()) return new ArrayList<>();
            return parse(ReadWriteFile.readFile(file));
        } catch (Throwable problem) {
            return new ArrayList<>();
        }
    }

    public static boolean save(List<double[]> points) {
        try {
            File file = AppUtil.getInstance().getSettingsFile(FILE_NAME);
            ReadWriteFile.writeFile(file, format(points));
            return load().size() == points.size();
        } catch (Throwable problem) {
            return false;
        }
    }
}
