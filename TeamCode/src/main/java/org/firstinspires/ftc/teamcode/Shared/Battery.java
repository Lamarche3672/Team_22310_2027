package org.firstinspires.ftc.teamcode.Shared;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.ArrayList;
import java.util.List;

/**
 * Watches the battery so the rest of the robot can go easy on it.
 *
 * Why this exists: our battery sags hard when the drive, the flywheels and the intake all pull
 * at once. Everything starting together can ask for 55 to 75 A from a battery behind a 20 A
 * fuse, and a worn pack drops several volts at that. Under about 7 V a hub shuts its motors
 * off, and a little lower it restarts in the middle of a match (that was the
 * "TargetPositionNotSetException" crash). Every mechanism asks this one object:
 *
 *   volts()         the latest reading
 *   compensation()  how much to scale power so a tired battery gives the same result
 *   powerLimit()    how much of full power is safe right now (1.0 = no limit)
 *   accelLimit()    how fast it is safe to ADD power right now (1.0 = normal)
 *
 * One Battery is made per OpMode and handed to every mechanism, so the hubs are only asked
 * for the voltage in one place. Nothing needs to "update" it: it refreshes itself whenever
 * somebody asks.
 *
 * Each hub has its own voltage sensor and they do not always agree: the Expansion Hub is fed
 * through the Control Hub and an XT30 cable, so it sees less. We read them all and go by the
 * LOWEST, because the hub with the least voltage is the one that will reset first.
 */
public final class Battery {
    /** What "full" means for voltage compensation. Motor specs are quoted at 12 V. */
    public static final double NOMINAL_VOLTS = 12.0;

    // Power limit: full power at or above FULL_POWER_VOLTS, easing down to MIN_POWER_LIMIT at
    // LOW_VOLTS. REV's hubs are rated down to 8 V and flag "battery low" at 7 V.
    public static final double FULL_POWER_VOLTS = 10.5;
    public static final double LOW_VOLTS = 7.5;
    public static final double MIN_POWER_LIMIT = 0.35;

    // The limit drops fast (a sag has to be caught now) and comes back slowly (so the robot
    // does not surge the moment the voltage recovers, sag again, and start oscillating).
    private static final double LIMIT_DROP_PER_SECOND = 5.0;
    private static final double LIMIT_RISE_PER_SECOND = 1.0;

    // Acceleration limit: adding power is what causes the big current spikes, so when the
    // battery is already under ACCEL_FULL_VOLTS we ramp up more gently.
    public static final double ACCEL_FULL_VOLTS = 11.0;
    public static final double ACCEL_SLOW_VOLTS = 9.0;
    public static final double MIN_ACCEL_LIMIT = 0.4;
    /**
     * While the flywheels are spinning up they are already pulling about 10 A. Anything else
     * that wants to accelerate in that half second does it at this fraction of its usual rate,
     * so the two current peaks do not land on top of each other.
     */
    public static final double ACCEL_WHILE_SPINNING_UP = 0.5;
    private static final long SPIN_UP_NOTE_NANOS = 150_000_000L;

    /** Hubs that differ by more than this have a bad power cable or connector between them. */
    public static final double HUB_DIFFERENCE_WARNING_VOLTS = 0.6;

    /** One voltage read is its own command to a hub (about 3 ms), so not every loop. */
    private static final long READ_NANOS = 50_000_000L;
    /** Smooths out single noisy readings before they move the limits or the compensation. */
    private static final double SMOOTHING_SECONDS = 0.15;

    /** A hub that has not given a good reading for this long is left out (it may have dropped off). */
    private static final long READING_FRESH_NANOS = 1_000_000_000L;
    /** The hubs are read 50 ms apart, so a quick sag looks like a difference. Only say so if it lasts. */
    private static final long HUBS_DIFFER_FOR_NANOS = 1_000_000_000L;

    private final List<VoltageSensor> sensors = new ArrayList<>();
    private final double[] readings;
    private long[] readingNanos = new long[0];
    private int nextSensor;
    private long hubsDifferSinceNanos;

    private double volts = Double.NaN;
    private double smoothedVolts = Double.NaN;
    private double lowestVolts = Double.NaN;
    private double powerLimit = 1.0;
    private long lastReadNanos;
    private long lastTickNanos;
    private long spinUpNotedNanos;

    public Battery(HardwareMap hardwareMap) {
        for (VoltageSensor sensor : hardwareMap.voltageSensor) sensors.add(sensor);
        readings = new double[sensors.size()];
        readingNanos = new long[sensors.size()];
        java.util.Arrays.fill(readings, Double.NaN);
    }

    public Battery(VoltageSensor sensor) {
        if (sensor != null) sensors.add(sensor);
        readings = new double[sensors.size()];
        readingNanos = new long[sensors.size()];
        java.util.Arrays.fill(readings, Double.NaN);
    }

    /** The lowest hub voltage right now, or NaN if no hub has given a usable reading yet. */
    public double volts() {
        tick();
        return volts;
    }

    public double lowestVolts() {
        tick();
        return lowestVolts;
    }

    /** 1.0 normally; lower when the battery is sagging. Multiply a mechanism's top power by this. */
    public double powerLimit() {
        tick();
        return powerLimit;
    }

    /** 1.0 normally; lower when the battery is low or the flywheels are spinning up. Multiply how fast power may RISE by this. */
    public double accelLimit() {
        tick();
        double limit = 1.0;
        if (!Double.isNaN(smoothedVolts)) {
            double fraction = (smoothedVolts - ACCEL_SLOW_VOLTS) / (ACCEL_FULL_VOLTS - ACCEL_SLOW_VOLTS);
            limit = MIN_ACCEL_LIMIT + (1.0 - MIN_ACCEL_LIMIT) * clamp(fraction, 0.0, 1.0);
        }
        if (System.nanoTime() - spinUpNotedNanos < SPIN_UP_NOTE_NANOS) limit *= ACCEL_WHILE_SPINNING_UP;
        return limit;
    }

    /** The flywheel controller calls this every loop it is limited by its spin-up current cap. */
    public void noteFlywheelSpinUp() {
        spinUpNotedNanos = System.nanoTime();
    }

    /**
     * How much to scale a motor power so the motor gets the same voltage it would get from a
     * full battery. 12 V battery -> 1.0, 10 V -> 1.2. Uses the smoothed voltage, so one dip
     * between two readings does not kick the flywheels, and is clamped so a bad reading
     * cannot do damage.
     */
    public double compensation() {
        tick();
        return Double.isNaN(smoothedVolts) ? 1.0 : clamp(NOMINAL_VOLTS / smoothedVolts, 0.8, 1.6);
    }

    /** Every hub's voltage, for the Driver Station, with a hint when they disagree. */
    public String describe() {
        tick();
        if (readings.length == 0) return "no voltage sensor";
        StringBuilder text = new StringBuilder();
        for (double reading : readings) {
            if (text.length() > 0) text.append(" / ");
            text.append(Double.isNaN(reading) ? "?" : String.format("%.1f", reading));
        }
        text.append(String.format("  (lowest %.1f)", lowestVolts));
        if (hubsDifferSinceNanos != 0 && System.nanoTime() - hubsDifferSinceNanos > HUBS_DIFFER_FOR_NANOS) {
            text.append("  HUBS DIFFER: check the XT30 cable between them");
        }
        return text.toString();
    }

    private void tick() {
        long now = System.nanoTime();
        if (!sensors.isEmpty() && now - lastReadNanos >= READ_NANOS) {
            lastReadNanos = now;
            // One hub per turn, so two hubs are each read ten times a second.
            double reading = sensors.get(nextSensor).getVoltage();
            // A failed read comes back as 0. Ignore it rather than panic the limiter.
            if (Double.isFinite(reading) && reading > 1.0) {
                readings[nextSensor] = reading;
                readingNanos[nextSensor] = now;
            }
            nextSensor = (nextSensor + 1) % sensors.size();

            // The lowest of the hubs that are still answering. A hub that sagged and then
            // dropped off must not hold the whole robot at its last low reading forever.
            double lowest = Double.NaN;
            double highest = Double.NaN;
            for (int i = 0; i < readings.length; i++) {
                if (Double.isNaN(readings[i]) || now - readingNanos[i] > READING_FRESH_NANOS) continue;
                if (!(lowest <= readings[i])) lowest = readings[i];
                if (!(highest >= readings[i])) highest = readings[i];
            }
            if (highest - lowest > HUB_DIFFERENCE_WARNING_VOLTS) {
                if (hubsDifferSinceNanos == 0) hubsDifferSinceNanos = now;
            } else {
                hubsDifferSinceNanos = 0;
            }
            if (!Double.isNaN(lowest)) {
                volts = lowest;
                if (!(lowestVolts <= lowest)) lowestVolts = lowest;
            }
        }

        // dt is capped so one slow loop cannot move the limit a long way in a single step.
        double dt = lastTickNanos == 0 ? 0.0 : Math.min((now - lastTickNanos) / 1e9, 0.05);
        lastTickNanos = now;
        if (Double.isNaN(volts)) return;

        smoothedVolts = Double.isNaN(smoothedVolts) ? volts
                : smoothedVolts + (volts - smoothedVolts) * Math.min(1.0, dt / SMOOTHING_SECONDS);
        double fraction = (smoothedVolts - LOW_VOLTS) / (FULL_POWER_VOLTS - LOW_VOLTS);
        double wanted = MIN_POWER_LIMIT + (1.0 - MIN_POWER_LIMIT) * clamp(fraction, 0.0, 1.0);
        powerLimit += clamp(wanted - powerLimit, -LIMIT_DROP_PER_SECOND * dt, LIMIT_RISE_PER_SECOND * dt);
    }

    private static double clamp(double value, double low, double high) {
        return Math.max(low, Math.min(high, value));
    }
}
