package org.firstinspires.ftc.teamcode.Shared;

import com.qualcomm.robotcore.exception.TargetPositionNotSetException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Runs the two shooter flywheels at a chosen speed. Teleop and autonomous both use this, so a
 * shot is the same shot in both.
 *
 * How it works, in order of importance:
 *
 *  1. Feedforward. A motor's speed is roughly proportional to the voltage it gets, so most of
 *     the power is simply "target speed / top speed". That part is corrected for the battery:
 *     a 10 V battery needs 20% more power than a 12 V one for the same speed.
 *  2. A small P and I correction on top, for friction and for the dip when a pollen goes through.
 *  3. A current limit. From rest a motor pulls its stall current, and two of them together are
 *     what used to drag the battery down. We never ask for more than SPIN_UP_VOLTS above what
 *     the wheel is already generating, which caps the current without slowing spin-up much.
 *  4. Both wheels get the same power, plus a small trim that pulls the slower one up.
 *
 * We drive the motors with plain power (RUN_WITHOUT_ENCODER) instead of the hub's built-in
 * velocity mode. The hub's own loop braked hard every time a wheel went slightly over speed,
 * and that brake-push-brake was the stutter we used to have.
 *
 * Speeds are in encoder ticks per second, the number DcMotorEx.getVelocity() gives.
 */
public final class FlywheelControl {

    // ---- the motor: REV HD Hex. The encoder is on the motor shaft (28 ticks/rev) and the bare
    // motor spins 6000 rpm at 12 V, so every HD Hex tops out at 2800 ticks/s whatever gearbox
    // is on it. Under load it can only HOLD about 85% of that.
    public static final double MAX_TICKS_PER_SECOND = 2800;
    public static final double HOLDABLE_TICKS_PER_SECOND = 0.85 * MAX_TICKS_PER_SECOND;

    /**
     * Which way "forwards" (shooting) is. Defined once here so teleop and autonomous can never
     * disagree again. If the wheels spin the wrong way on the robot, flip this one line.
     */
    public static final DcMotorSimple.Direction DIRECTION = DcMotorSimple.Direction.FORWARD;

    // ---- gains. F = 1.0 means "exactly the theoretical feedforward". P is power per 1000
    // ticks/s of error, I is power per 1000 ticks/s of error held for one second.
    // P = 0.7 came from a sweep against a model of the hub's speed reading (which lags about
    // 25 ms and moves in steps of 20 ticks/s): higher overshoots a light wheel, lower is slow
    // to recover after a pollen. If the real wheels hunt up and down, lower P first.
    public static final double DEFAULT_P = 0.7;
    public static final double DEFAULT_F = 1.0;
    public static final double DEFAULT_I = 0.5;

    // ---- current limit while spinning up (see point 3 above). On a healthy battery we allow
    // 7 V across the motor, about 5 A each. Under 11 V that eases down to 3.5 V by 9.5 V.
    public static final double SPIN_UP_VOLTS = 7.0;
    public static final double LOW_BATTERY_SPIN_UP_VOLTS = 3.5;
    public static final double FULL_SPIN_UP_ABOVE_VOLTS = 11.0;
    public static final double GENTLE_SPIN_UP_BELOW_VOLTS = 9.5;

    // ---- "is it ready to shoot?"
    /** Within this many ticks/s of the target counts as at speed. */
    public static final double READY_TOLERANCE = 75;
    /** It has to stay at speed this long, so one lucky reading on the way past does not count. */
    public static final double READY_HOLD_MS = 100;
    // A wheel that cannot reach the target (tired battery, target set too high) would never be
    // "at speed". If it has stopped speeding up (speed stays inside LEVEL_BAND for LEVEL_MS)
    // and is at least half way there, that is as good as it gets, so we call it ready too.
    public static final double LEVEL_BAND = 120;
    public static final double LEVEL_MS = 300;
    /** If the wheels never get ready, feed anyway after this long rather than hold the pollen forever. */
    public static final double SPIN_UP_TIMEOUT_MS = 2500;
    /** After a pollen slows the wheels, the feed waits at most this long for them to recover. */
    public static final double MAX_FEED_WAIT_MS = 700;

    private static final double INTEGRATE_WITHIN = 200;
    private static final double MAX_INTEGRAL_POWER = 0.1;
    // Matching the two wheels. The hub reports speed in steps of about 20 ticks/s, so the raw
    // difference between the wheels flickers. The trim works on a smoothed difference: it is
    // there to take out a steady mismatch, not to react to every reading.
    private static final double MATCH_GAIN = 1.5;
    private static final double MAX_MATCH_POWER = 0.05;
    private static final double MATCH_SMOOTHING_SECONDS = 0.25;
    private static final double DEAD_ENCODER_BELOW = 50;
    /** Skip motor writes that change the power by less than this; each write costs loop time. */
    private static final double WRITE_THRESHOLD = 0.005;
    /** Re-send the power this often even if unchanged, so a hub that restarted gets it again. */
    private static final long RESEND_NANOS = 250_000_000L;
    private static final long IDLE_READ_NANOS = 200_000_000L;

    private final DcMotorEx right;
    private final DcMotorEx left;
    private final Battery battery;

    private double p = DEFAULT_P;
    private double f = DEFAULT_F;
    private double i = DEFAULT_I;

    private double target;
    private double power;
    private double rightPower;
    private double leftPower;
    private double sentRight = Double.NaN;
    private double sentLeft = Double.NaN;
    private double integral;
    private double rightVelocity;
    private double leftVelocity;
    /** Smoothed (right - left) speed, used to trim the wheels toward each other. */
    private double mismatch;
    private long lastUpdateNanos;
    private long lastWriteNanos;
    private long lastReadNanos;

    // readiness bookkeeping
    private boolean atSpeed;
    private long atSpeedSinceNanos;
    private long lastAtSpeedNanos;
    private double levelLowest;
    private double levelHighest;
    private long levelSinceNanos;
    private long spinStartNanos;
    private double feedReference = Double.NaN;
    private long feedWaitSinceNanos;

    public FlywheelControl(HardwareMap hardwareMap, Battery battery) {
        right = hardwareMap.get(DcMotorEx.class, "right_flywheel");
        left = hardwareMap.get(DcMotorEx.class, "left_flywheel");
        this.battery = battery;
        right.setDirection(DIRECTION);
        left.setDirection(DIRECTION);
        configure();
        stop();
    }

    private void configure() {
        for (DcMotorEx motor : new DcMotorEx[] {right, left}) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // FLOAT, not BRAKE: when we stop commanding the wheels they should coast, not slam.
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public void setGains(double p, double f, double i) {
        this.p = Math.max(0, p);
        this.f = Math.max(0, f);
        this.i = Math.max(0, i);
    }

    /** Ticks per second. Negative runs the wheels backwards (clearing a jam). 0 = stop(). */
    public void setTarget(double ticksPerSecond) {
        if (!Double.isFinite(ticksPerSecond)) ticksPerSecond = 0;
        if (ticksPerSecond == target) return;
        if (Math.signum(ticksPerSecond) != Math.signum(target)) integral = 0;

        // A small nudge to the speed (the camera distance moving a little) keeps the "ready"
        // state. A real change means ready has to be earned again from scratch.
        boolean nudge = target != 0 && Math.signum(ticksPerSecond) == Math.signum(target)
                && Math.abs(ticksPerSecond - target) <= READY_TOLERANCE;
        if (nudge) {
            if (feedReference == target) feedReference = ticksPerSecond;
        } else {
            spinStartNanos = System.nanoTime();
            atSpeed = false;
            feedReference = Double.NaN;
            feedWaitSinceNanos = 0;
        }
        target = ticksPerSecond;
    }

    /** Lets the wheels coast to a stop. */
    public void stop() {
        target = 0;
        integral = 0;
        power = 0;
        rightPower = 0;
        leftPower = 0;
        atSpeed = false;
        feedReference = Double.NaN;
        feedWaitSinceNanos = 0;
        write(0, 0, true);
    }

    /** Call once every loop. Reads the wheel speeds and sends the new motor powers. */
    public void update() {
        long now = System.nanoTime();
        double dt = lastUpdateNanos == 0 ? 0.0 : Math.min(0.1, (now - lastUpdateNanos) / 1e9);
        lastUpdateNanos = now;

        // While stopped the speed only matters for telemetry and for "are they still spinning
        // the other way?", so a few reads a second is plenty.
        if (target != 0 || now - lastReadNanos >= IDLE_READ_NANOS) {
            rightVelocity = right.getVelocity();
            leftVelocity = left.getVelocity();
            lastReadNanos = now;
        }

        double measured = speed();
        power = target == 0 ? 0 : nextPower(measured, dt);
        updateMismatch(dt);
        // mismatch is "how far the right wheel is ahead of the left".
        rightPower = matched(power, mismatch);
        leftPower = matched(power, -mismatch);
        boolean changed = Math.abs(rightPower - sentRight) >= WRITE_THRESHOLD
                || Math.abs(leftPower - sentLeft) >= WRITE_THRESHOLD
                || (power == 0 && (sentRight != 0 || sentLeft != 0));
        write(rightPower, leftPower, changed || now - lastWriteNanos >= RESEND_NANOS);

        trackReadiness(measured, now);
    }

    /**
     * The speed we control: the average of the two wheels. If one encoder is dead (reads 0
     * while the other is clearly spinning) we go by the live one, so a loose encoder cable
     * costs us nothing.
     */
    public double speed() {
        double r = rightVelocity;
        double l = leftVelocity;
        if (Math.abs(r) < DEAD_ENCODER_BELOW && Math.abs(l) > 4 * DEAD_ENCODER_BELOW) return l;
        if (Math.abs(l) < DEAD_ENCODER_BELOW && Math.abs(r) > 4 * DEAD_ENCODER_BELOW) return r;
        return 0.5 * (r + l);
    }

    private double nextPower(double measured, double dt) {
        double direction = Math.signum(target);
        double error = target - measured;

        // Only integrate near the target. Integrating during the whole spin-up would wind the
        // term up and overshoot.
        if (Math.abs(error) < INTEGRATE_WITHIN) integral += error * dt;
        double integralPower = i / 1000.0 * integral;
        if (Math.abs(integralPower) > MAX_INTEGRAL_POWER) {
            integralPower = Math.signum(integralPower) * MAX_INTEGRAL_POWER;
            integral = integralPower * 1000.0 / Math.max(i, 1e-9);
        }

        double wanted = (f * target / MAX_TICKS_PER_SECOND) * battery.compensation()
                + p / 1000.0 * error + integralPower;
        // Never push against the direction we want to spin; over speed we just ease off.
        wanted = direction > 0 ? Math.max(0, wanted) : Math.min(0, wanted);

        // Current limit. A spinning motor generates its own voltage ("back-EMF"); the current
        // is set by how far the voltage we apply is from that. Staying within `headroom` of it
        // caps the current both when speeding up and when slowing down.
        double supply = Double.isNaN(battery.volts()) ? Battery.NOMINAL_VOLTS : battery.volts();
        double backEmf = measured / MAX_TICKS_PER_SECOND * Battery.NOMINAL_VOLTS / supply;
        double headroom = spinUpVolts(supply) / supply;
        // Still held back by the current cap = still spinning up. Tell the battery guard, so
        // the drive and intake go a little easier until the wheels are up.
        if (Math.abs(wanted) > Math.abs(backEmf) + headroom) battery.noteFlywheelSpinUp();
        wanted = Math.max(backEmf - headroom, Math.min(backEmf + headroom, wanted));
        return Math.max(-1.0, Math.min(1.0, wanted));
    }

    /** How many volts above the wheel's own back-EMF we allow, given the battery voltage. */
    public static double spinUpVolts(double supply) {
        if (!Double.isFinite(supply)) return SPIN_UP_VOLTS;
        double fraction = (supply - GENTLE_SPIN_UP_BELOW_VOLTS) / (FULL_SPIN_UP_ABOVE_VOLTS - GENTLE_SPIN_UP_BELOW_VOLTS);
        fraction = Math.max(0, Math.min(1, fraction));
        return LOW_BATTERY_SPIN_UP_VOLTS + (SPIN_UP_VOLTS - LOW_BATTERY_SPIN_UP_VOLTS) * fraction;
    }

    /**
     * The same power on both wheels leaves them at slightly different speeds if one has more
     * friction, and unequal wheels curve the shot. The trim gives the slower wheel a little
     * more and the faster one a little less. Skipped when an encoder is dead, because then
     * "the difference" is meaningless.
     */
    private void updateMismatch(double dt) {
        boolean bothAlive = Math.abs(rightVelocity) >= DEAD_ENCODER_BELOW && Math.abs(leftVelocity) >= DEAD_ENCODER_BELOW;
        if (target == 0 || !bothAlive) {
            mismatch = 0;
            return;
        }
        mismatch += (rightVelocity - leftVelocity - mismatch) * Math.min(1.0, dt / MATCH_SMOOTHING_SECONDS);
    }

    /** aheadBy: how far this wheel is ahead of the other (ticks/s, smoothed); negative if behind. */
    private double matched(double common, double aheadBy) {
        if (common == 0) return 0;
        double trim = -MATCH_GAIN / 1000.0 * 0.5 * aheadBy * Math.signum(target);
        trim = Math.max(-MAX_MATCH_POWER, Math.min(MAX_MATCH_POWER, trim));
        double value = common + trim;
        value = target > 0 ? Math.max(0, value) : Math.min(0, value);
        return Math.max(-1.0, Math.min(1.0, value));
    }

    private void trackReadiness(double measured, long now) {
        boolean inBand = target > 0 && Math.abs(target - measured) <= READY_TOLERANCE;
        if (inBand && !atSpeed) atSpeedSinceNanos = now;
        if (inBand) lastAtSpeedNanos = now;
        atSpeed = inBand;

        levelLowest = Math.min(levelLowest, measured);
        levelHighest = Math.max(levelHighest, measured);
        if (target <= 0 || levelHighest - levelLowest > LEVEL_BAND) {
            levelLowest = measured;
            levelHighest = measured;
            levelSinceNanos = now;
        }

        // Remember what "ready" looked like, so the feed knows what speed to wait for after
        // each pollen: the target if we reach it, otherwise wherever the wheels levelled off.
        if (atSpeed && now - atSpeedSinceNanos >= READY_HOLD_MS * 1e6) {
            feedReference = target;
        } else if (!atSpeed && now - lastAtSpeedNanos >= LEVEL_MS * 1e6 && hasLevelledOff(measured, now)) {
            // Steady, but short of the target for a while: this is all the wheels have right
            // now (tired battery, or a target set too high). Shoot at this speed rather than
            // not at all. The wait stops a normal dip after a pollen from lowering the bar.
            feedReference = Math.min(target, 0.5 * (levelLowest + levelHighest));
        }
    }

    private boolean hasLevelledOff(double measured, long now) {
        return target > 0 && measured >= 0.5 * target && now - levelSinceNanos >= LEVEL_MS * 1e6;
    }

    /** True right now, this loop. Use isReady() to decide when to shoot. */
    public boolean isAtSpeed() {
        return atSpeed;
    }

    /** Steady at the target, or levelled off as fast as it will go. Safe to start feeding. */
    public boolean isReady() {
        long now = System.nanoTime();
        return (atSpeed && now - atSpeedSinceNanos >= READY_HOLD_MS * 1e6) || hasLevelledOff(speed(), now);
    }

    /**
     * How hard the index should feed right now, 0 to 1. This is what makes shots consistent:
     * every pollen slows the wheels a little, and the next one should not arrive until they
     * are back. 1 = wheels are ready, feed normally. 0 = wait. In between it eases, so the
     * belt slows down and speeds up instead of banging on and off.
     *
     * It never blocks for long: if the wheels cannot get ready (dead encoder, flat battery)
     * it gives up waiting and lets the pollen through anyway.
     */
    public double feedScale() {
        if (target <= 0) return 1.0;
        long now = System.nanoTime();
        if (Double.isNaN(feedReference)) {
            return now - spinStartNanos > SPIN_UP_TIMEOUT_MS * 1e6 ? 1.0 : 0.0;
        }
        double shortfall = feedReference - speed();
        double scale = Math.max(0.0, Math.min(1.0, 1.0 - (shortfall - READY_TOLERANCE) / READY_TOLERANCE));
        if (scale >= 1.0) {
            feedWaitSinceNanos = 0;
            return 1.0;
        }
        if (feedWaitSinceNanos == 0) feedWaitSinceNanos = now;
        return now - feedWaitSinceNanos > MAX_FEED_WAIT_MS * 1e6 ? 1.0 : scale;
    }

    public double target() {
        return target;
    }

    public double rightVelocity() {
        return rightVelocity;
    }

    public double leftVelocity() {
        return leftVelocity;
    }

    public double power() {
        return power;
    }

    /** True when one wheel reads (almost) nothing while we are clearly driving it. */
    public boolean encoderLooksDead() {
        return Math.abs(power) > 0.2
                && (Math.abs(rightVelocity) < DEAD_ENCODER_BELOW || Math.abs(leftVelocity) < DEAD_ENCODER_BELOW);
    }

    private void write(double rightValue, double leftValue, boolean send) {
        if (!send) return;
        try {
            right.setPower(rightValue);
            left.setPower(leftValue);
        } catch (TargetPositionNotSetException hubReset) {
            // The hub restarted and forgot the motor setup. Send it again and retry once.
            HubReset.forget(right, left);
            try {
                configure();
                right.setPower(rightValue);
                left.setPower(leftValue);
            } catch (TargetPositionNotSetException stillDown) {
                return;
            }
        }
        sentRight = rightValue;
        sentLeft = leftValue;
        lastWriteNanos = System.nanoTime();
    }
}
