package org.firstinspires.ftc.teamcode.Shared;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * Watches the battery so the rest of the robot can go easy on it.
 *
 * Why this exists: our battery sags hard when the drive, the flywheels and the intake all pull
 * at once. Go low enough and a hub resets in the middle of a match (that was the
 * "TargetPositionNotSetException" crash). Every mechanism asks this one object:
 *
 *   volts()         the latest reading, for voltage compensation (same shot on any battery)
 *   powerLimit()    how much of full power is safe right now (1.0 = no limit)
 *   accelLimit()    how fast it is safe to ADD power right now (1.0 = normal)
 *
 * One Battery is made per OpMode and handed to every mechanism, so the hub is only asked for
 * the voltage in one place. Nothing needs to "update" it: it refreshes itself whenever
 * somebody asks, at most every READ_MS.
 */
public final class Battery {
    /** What "full" means for voltage compensation. Motor specs are quoted at 12 V. */
    public static final double NOMINAL_VOLTS = 12.0;

    // Power limit: full power at or above FULL_POWER_VOLTS, easing down to MIN_POWER_LIMIT at
    // LOW_VOLTS. The hubs start to misbehave somewhere under 7 V, so 7.5 leaves a margin.
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

    /** One voltage read is a separate command to the hub (a few ms), so not every loop. */
    private static final long READ_NANOS = 50_000_000L;
    /** Smooths out single noisy readings before they move the limit. */
    private static final double SMOOTHING_SECONDS = 0.2;

    private final VoltageSensor sensor;

    private double volts = Double.NaN;
    private double smoothedVolts = Double.NaN;
    private double lowestVolts = Double.NaN;
    private double powerLimit = 1.0;
    private long lastReadNanos;
    private long lastTickNanos;

    public Battery(HardwareMap hardwareMap) {
        this(hardwareMap.voltageSensor.iterator().hasNext() ? hardwareMap.voltageSensor.iterator().next() : null);
    }

    public Battery(VoltageSensor sensor) {
        this.sensor = sensor;
    }

    /** Latest battery voltage, or NaN if the hub has not given a usable reading yet. */
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

    /** 1.0 normally; lower when the battery is low. Multiply how fast power may RISE by this. */
    public double accelLimit() {
        tick();
        if (Double.isNaN(smoothedVolts)) return 1.0;
        double fraction = (smoothedVolts - ACCEL_SLOW_VOLTS) / (ACCEL_FULL_VOLTS - ACCEL_SLOW_VOLTS);
        return MIN_ACCEL_LIMIT + (1.0 - MIN_ACCEL_LIMIT) * clamp(fraction, 0.0, 1.0);
    }

    /**
     * How much to scale a motor power so the motor gets the same voltage it would get from a
     * full battery. 12 V battery -> 1.0, 10 V -> 1.2. Clamped so a bad reading cannot do damage.
     */
    public double compensation() {
        tick();
        return Double.isNaN(volts) ? 1.0 : clamp(NOMINAL_VOLTS / volts, 0.8, 1.6);
    }

    private void tick() {
        long now = System.nanoTime();
        if (sensor != null && now - lastReadNanos >= READ_NANOS) {
            lastReadNanos = now;
            double reading = sensor.getVoltage();
            // A failed read comes back as 0. Ignore it rather than panic the limiter.
            if (Double.isFinite(reading) && reading > 1.0) {
                volts = reading;
                if (!(lowestVolts <= reading)) lowestVolts = reading;
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
