package org.firstinspires.ftc.teamcode.Shared;

import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.Locale;

/**
 * The shooter's numbers: the flywheel speed, the three control gains and the two feed times
 * (how long the belt runs for one pollen, and how long it then waits). The drive team
 * changes them from gamepad 1 with the D-PAD (see Shooter) and they are kept in a small file
 * on the Control Hub, so a tune found on the practice field is still there after a restart
 * and after new code is installed. Autonomous reads the same file, so the autos always shoot
 * exactly the way teleop does.
 *
 * There is ONE speed. It does not change with the camera's distance to the hive any more:
 * that was tried, and a wrong saved number made every shot fall short with nobody able to
 * see why. One number on the screen that only the D-PAD changes cannot do that.
 *
 * About the speed number: it is what the motor encoders report (ticks per second), which is
 * the number the team has always dialled in. These motors read about 2800 flat out, so
 * anything above that, such as the default 3000, simply means "everything they have".
 */
public final class ShooterSettings {
    public static final double DEFAULT_SPEED = 3000;
    public static final double MIN_SPEED = 400;
    public static final double MAX_SPEED = 3200;
    public static final double MAX_GAIN = 3.0;
    /** F is a fraction of the theoretical feedforward. Past 1.5 it would just pin the motors at full. */
    public static final double MAX_F = 1.5;
    // Feeding (see FeedPulser). By default the belt simply runs for as long as LEFT BUMPER is
    // held: OFF is 0, which means no pauses. Raising OFF on the menu turns on "one pollen at
    // a time": belt on for ON ms, then off for OFF ms, and so on while the bumper is held.
    public static final double DEFAULT_FEED_ON_MS = 250;
    public static final double DEFAULT_FEED_OFF_MS = 0;
    public static final double MAX_FEED_MS = 1500;

    // "_v2": the first file could be changed by brushing the D-PAD at any moment, and shots
    // went short without anybody having meant to lower the speed. That file is not read any
    // more, so every number starts again from its default (speed 3000 = full power). The
    // menu now has to be opened with BACK before the D-PAD changes anything.
    private static final String FILE_NAME = "shooter_settings_v2.txt";

    public double speed = DEFAULT_SPEED;
    public double p = FlywheelControl.DEFAULT_P;
    public double i = FlywheelControl.DEFAULT_I;
    public double f = FlywheelControl.DEFAULT_F;
    public double feedOnMs = DEFAULT_FEED_ON_MS;
    public double feedOffMs = DEFAULT_FEED_OFF_MS;

    public void resetToDefaults() {
        speed = DEFAULT_SPEED;
        p = FlywheelControl.DEFAULT_P;
        i = FlywheelControl.DEFAULT_I;
        f = FlywheelControl.DEFAULT_F;
        feedOnMs = DEFAULT_FEED_ON_MS;
        feedOffMs = DEFAULT_FEED_OFF_MS;
    }

    /** Keeps every number somewhere sensible, whatever was typed, saved or read back. */
    public void clamp() {
        speed = within(speed, MIN_SPEED, MAX_SPEED, DEFAULT_SPEED);
        p = within(p, 0, MAX_GAIN, FlywheelControl.DEFAULT_P);
        i = within(i, 0, MAX_GAIN, FlywheelControl.DEFAULT_I);
        f = within(f, 0, MAX_F, FlywheelControl.DEFAULT_F);
        // The belt needs at least a moment to move a pollen at all; 0 off = no pauses.
        feedOnMs = within(feedOnMs, 50, MAX_FEED_MS, DEFAULT_FEED_ON_MS);
        feedOffMs = within(feedOffMs, 0, MAX_FEED_MS, DEFAULT_FEED_OFF_MS);
    }

    private static double within(double value, double low, double high, double ifNonsense) {
        if (!Double.isFinite(value)) return ifNonsense;
        return Math.max(low, Math.min(high, value));
    }

    public String describe() {
        String feed = feedOffMs <= 0 ? "feed runs while held" : String.format(Locale.US, "feed %.0f on / %.0f off", feedOnMs, feedOffMs);
        return String.format(Locale.US, "speed %.0f, P %.2f, I %.2f, F %.2f, %s", speed, p, i, f, feed);
    }

    /** One line: "speed p i f feedOn feedOff". */
    public String format() {
        return String.format(Locale.US, "%.0f %.3f %.3f %.3f %.0f %.0f\n", speed, p, i, f, feedOnMs, feedOffMs);
    }

    /** Reads a line written by format(). Anything missing or unreadable keeps its default. */
    public static ShooterSettings parse(String text) {
        ShooterSettings settings = new ShooterSettings();
        if (text == null) return settings;
        String[] parts = text.trim().split("\\s+");
        try {
            if (parts.length > 0) settings.speed = Double.parseDouble(parts[0]);
            if (parts.length > 1) settings.p = Double.parseDouble(parts[1]);
            if (parts.length > 2) settings.i = Double.parseDouble(parts[2]);
            if (parts.length > 3) settings.f = Double.parseDouble(parts[3]);
            if (parts.length > 4) settings.feedOnMs = Double.parseDouble(parts[4]);
            if (parts.length > 5) settings.feedOffMs = Double.parseDouble(parts[5]);
        } catch (NumberFormatException damaged) {
            settings.resetToDefaults();
        }
        settings.clamp();
        return settings;
    }

    // Reading and writing must never be able to stop an OpMode, so every kind of failure
    // (no file yet, damaged file, not running on a robot) just means "use the defaults".
    public static ShooterSettings load() {
        try {
            File file = AppUtil.getInstance().getSettingsFile(FILE_NAME);
            if (!file.exists()) return new ShooterSettings();
            return parse(ReadWriteFile.readFile(file));
        } catch (Throwable problem) {
            return new ShooterSettings();
        }
    }

    public boolean save() {
        try {
            clamp();
            File file = AppUtil.getInstance().getSettingsFile(FILE_NAME);
            ReadWriteFile.writeFile(file, format());
            // Read it straight back: the only way to know the write really worked.
            ShooterSettings back = parse(ReadWriteFile.readFile(file));
            return Math.abs(back.speed - speed) < 1 && Math.abs(back.p - p) < 0.002
                    && Math.abs(back.i - i) < 0.002 && Math.abs(back.f - f) < 0.002
                    && Math.abs(back.feedOnMs - feedOnMs) < 1 && Math.abs(back.feedOffMs - feedOffMs) < 1;
        } catch (Throwable problem) {
            return false;
        }
    }
}
