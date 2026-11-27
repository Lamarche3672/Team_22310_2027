package org.firstinspires.ftc.teamcode.Shared;

/**
 * Decides when the index belt runs while the feed button is held.
 *
 * By default (OFF time 0) that is simply: all the time the button is held. Hold LEFT BUMPER
 * and the belt runs; let go and it stops.
 *
 * It can also feed one pollen at a time, which is switched on by giving OFF a value on the
 * gamepad 1 menu. Why that exists: with the belt simply running, the pollen arrive nose to tail. The first one is a
 * great shot. It also takes speed out of the wheels, and the second is already touching
 * them before they have it back, so two or three get spat out weakly together. One pollen
 * on its own shoots well, so every pollen is made to be "one on its own": the belt runs just
 * long enough to hand one over, stops while the wheels get their speed back, then hands over
 * the next.
 *
 *   belt on for ON ms  ->  belt off for OFF ms  ->  on  ->  off  ...
 *
 * If the wheels have a working speed reading, the off time also waits until they are back
 * near the speed they had before the first pollen (never more than MAX_EXTRA_WAIT_MS longer,
 * so a wheel that cannot recover never stalls the volley). With no reading it goes purely
 * by the clock.
 *
 * Teleop and autonomous both use this, so a volley looks the same in both. The two times are
 * on the gamepad 1 menu (ShooterSettings).
 */
public final class FeedPulser {
    /** How close to their starting speed the wheels have to be before the next pollen goes. */
    private static final double RECOVERED_WITHIN = 100;
    private static final double MAX_EXTRA_WAIT_MS = 500;

    private double onMs = ShooterSettings.DEFAULT_FEED_ON_MS;
    private double offMs = ShooterSettings.DEFAULT_FEED_OFF_MS;

    private boolean feeding;
    private boolean beltOn;
    private long phaseStartNanos;
    /** Wheel speed just before the first pollen of this press, or NaN when there was none to read. */
    private double speedBefore = Double.NaN;

    public void setTimes(double onMs, double offMs) {
        this.onMs = Math.max(0, onMs);
        this.offMs = Math.max(0, offMs);
    }

    /**
     * Call every loop. wantFeed = the driver is holding the feed button (or the auto is
     * feeding). wheels may be null. Returns true while the belt should be running.
     */
    public boolean update(boolean wantFeed, FlywheelControl wheels, long now) {
        if (!wantFeed) {
            feeding = false;
            beltOn = false;
            return false;
        }
        boolean reading = wheels != null && wheels.hasSpeedReading();
        if (!feeding) {
            // The first pollen goes the moment the button is pressed.
            feeding = true;
            beltOn = true;
            phaseStartNanos = now;
            speedBefore = reading ? wheels.speed() : Double.NaN;
            return true;
        }
        if (offMs <= 0) return true;

        double inPhaseMs = (now - phaseStartNanos) / 1e6;
        if (beltOn) {
            if (inPhaseMs >= onMs) {
                beltOn = false;
                phaseStartNanos = now;
            }
        } else if (inPhaseMs >= offMs) {
            boolean recovered = !reading || Double.isNaN(speedBefore) || wheels.speed() >= speedBefore - RECOVERED_WITHIN;
            if (recovered || inPhaseMs >= offMs + MAX_EXTRA_WAIT_MS) {
                beltOn = true;
                phaseStartNanos = now;
            }
        }
        return beltOn;
    }
}
