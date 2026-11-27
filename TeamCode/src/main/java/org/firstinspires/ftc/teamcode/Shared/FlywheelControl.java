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
 * Points 2 to 4 all lean on the wheel encoders, and those are the first thing to go wrong on a
 * real robot. So the encoders are checked against what the motors are being told to do:
 *   - reads backwards (encoder and motor wired opposite ways): the reading is flipped
 *   - one reads nothing: the other one is used for both
 *   - neither reads anything: the wheels run on feedforward alone, eased in over time
 * Before this check a bad reading quietly held the wheels at a fraction of their speed.
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
    // is on it. A target above that is allowed: it just means full power.
    public static final double MAX_TICKS_PER_SECOND = 2800;
    // A target at or above the top speed means "flat out": the motors are simply asked for
    // full power, with no speed correction and no wheel matching (both could only take power
    // away). So a weak shot at flat out is never the speed control holding the wheels back.
    //
    // What flat out does NOT skip is the soft start. For a day it did: full power went to both
    // motors a third of a second after the button, each pulled its full stall current, and
    // with the drive moving at the same moment the battery fell to 5.3 V and both hubs
    // restarted in the middle of a run. The spin-up current cap below costs a few tenths of
    // a second on the way up and nothing once the wheels are turning.

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

    // ---- current limit while spinning up (see point 3 above). At 11 V we allow 7 V across
    // the motor, about 5 A each. A fresh battery (12.5 V and up) gets 9 V, about 6.5 A each,
    // for a quicker spin-up. Under 11 V it eases down to 3.5 V by 9.5 V. The voltage is read
    // while the wheels are pulling current, so a battery that sags gets less straight away.
    public static final double STRONG_BATTERY_SPIN_UP_VOLTS = 9.0;
    public static final double SPIN_UP_VOLTS = 7.0;
    public static final double LOW_BATTERY_SPIN_UP_VOLTS = 3.5;
    public static final double STRONG_BATTERY_ABOVE_VOLTS = 12.5;
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
    /** In autonomous the index is held for the wheels at the start of a volley, at most this long. */
    public static final double MAX_FEED_WAIT_MS = 600;

    /**
     * Extra power while the index is feeding. A pollen going through takes speed out of the
     * wheels, and the index motor starting pulls the battery down a little, and both happen
     * at the same moment. Waiting for the speed to drop before reacting is too late, so the
     * power goes up as soon as the wheels are even slightly down while feeding, well before
     * the normal correction would have reacted that hard. It is never added at or above the
     * target, so it cannot make a pollen leave faster than it should.
     */
    public static final double FEED_BOOST_POWER = 0.08;
    private static final double FEED_BOOST_BELOW_TICKS = 20;
    private static final double INTEGRATE_WITHIN = 200;
    private static final double MAX_INTEGRAL_POWER = 0.1;
    // Matching the two wheels. Unequal wheels put spin on a pollen and it curves, so the two
    // are held within MATCHED_WITHIN of each other. The hub reports speed in steps of about
    // 20 ticks/s, so the raw difference flickers; the trim works on a smoothed difference.
    // Two parts: a quick one in proportion to the difference, and a slow one that keeps
    // building until the difference is gone. The slow part is what closes the last gap: a
    // steady mismatch (one wheel with more friction) is otherwise only ever made smaller.
    public static final double MATCHED_WITHIN = 50;
    private static final double MATCH_GAIN = 1.5;
    /** Power moved from the fast wheel to the slow one, per 1000 ticks/s apart, per second. */
    private static final double MATCH_BUILD_GAIN = 3.0;
    /** The most power the trim may move. At full power only the fast wheel can change (down). */
    private static final double MAX_MATCH_POWER = 0.2;
    private static final double MATCH_SMOOTHING_SECONDS = 0.25;
    private static final double DEAD_ENCODER_BELOW = 50;
    // ---- checking the encoders against the motors
    /** "Being driven" for these checks: enough power that a healthy wheel is certainly turning. */
    private static final double DRIVEN_POWER = 0.1;
    // Driven one way but reading this fast the other way, for this long: that encoder counts
    // backwards. A wheel cannot really do that, so there is nothing else it could be.
    private static final double BACKWARDS_ABOVE = 300;
    private static final double BACKWARDS_SECONDS = 0.4;
    /** Both wheels driven and both reading nothing for this long: no encoders, run without them. */
    private static final double NO_READING_SECONDS = 0.5;
    /** Without a speed reading the current cannot be capped, so the power is eased in instead (per second). */
    private static final double BLIND_POWER_RISE_PER_SECOND = 1.2;
    /** Without a speed reading "ready" can only go by the clock. */
    public static final double BLIND_READY_MS = 1500;
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
    /** The slow part of the trim: power taken off the right wheel and given to the left (signed). */
    private double matchBuilt;
    // Encoder health. Kept for the whole OpMode: wiring does not fix itself between shots.
    private double rightEncoderSign = 1;
    private double leftEncoderSign = 1;
    private double rightBackwardsSeconds;
    private double leftBackwardsSeconds;
    private double noReadingSeconds;
    private boolean blind;
    private boolean feeding;
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
    /** True once feedScale() has let the index go; it then stays on for the rest of the volley. */
    private boolean feedStarted;

    public FlywheelControl(HardwareMap hardwareMap, Battery battery) {
        this(hardwareMap.get(DcMotorEx.class, "right_flywheel"),
                hardwareMap.get(DcMotorEx.class, "left_flywheel"), battery);
    }

    FlywheelControl(DcMotorEx right, DcMotorEx left, Battery battery) {
        this.right = right;
        this.left = left;
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
            feedStarted = false;
            // "Steady for a while" has to start again too. Otherwise wheels sitting at the old
            // speed would count as levelled off for the new one the instant it is set.
            levelSinceNanos = spinStartNanos;
            levelLowest = speed();
            levelHighest = levelLowest;
            lastAtSpeedNanos = spinStartNanos;
        }
        target = ticksPerSecond;
    }

    /** Tell the wheels the index is pushing pollen at them right now (see FEED_BOOST_POWER). Set every loop. */
    public void setFeeding(boolean feeding) {
        this.feeding = feeding;
    }

    /** Call when nobody is feeding, so time spent idle does not count as "waited long enough". */
    public void cancelFeedWait() {
        feedWaitSinceNanos = 0;
        feedStarted = false;
    }

    /** Lets the wheels coast to a stop. */
    public void stop() {
        target = 0;
        integral = 0;
        power = 0;
        feeding = false;
        feedStarted = false;
        matchBuilt = 0;
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
            rightVelocity = rightEncoderSign * right.getVelocity();
            leftVelocity = leftEncoderSign * left.getVelocity();
            lastReadNanos = now;
        }
        checkEncoders(dt);

        double measured = speed();
        power = target == 0 ? 0 : nextPower(measured, dt);
        updateMismatch(dt);
        // mismatch is "how far the right wheel is ahead of the left".
        // Flat out both wheels get the same full power. Matching them there could only mean
        // slowing the faster one down, which throws away exactly the speed that was asked for.
        boolean flatOut = isFlatOut();
        rightPower = flatOut ? power : matched(power, mismatch, matchBuilt);
        leftPower = flatOut ? power : matched(power, -mismatch, -matchBuilt);
        boolean changed = Math.abs(rightPower - sentRight) >= WRITE_THRESHOLD
                || Math.abs(leftPower - sentLeft) >= WRITE_THRESHOLD
                || (power == 0 && (sentRight != 0 || sentLeft != 0));
        write(rightPower, leftPower, changed || now - lastWriteNanos >= RESEND_NANOS);

        trackReadiness(measured, now);
    }

    /** True when there is a real speed to go by: an encoder is answering and the wheels are turning. */
    public boolean hasSpeedReading() {
        return !blind && target != 0 && Math.abs(speed()) > 4 * DEAD_ENCODER_BELOW;
    }

    /**
     * Compares what the encoders say with what the motors were last told to do, and deals with
     * the two ways they go wrong.
     */
    private void checkEncoders(double dt) {
        // An encoder that counts backwards makes the controller think the wheel is going the
        // wrong way, and the current cap then holds it at about a quarter of full power.
        rightBackwardsSeconds = readsBackwards(rightVelocity, sentRight) ? rightBackwardsSeconds + dt : 0;
        leftBackwardsSeconds = readsBackwards(leftVelocity, sentLeft) ? leftBackwardsSeconds + dt : 0;
        if (rightBackwardsSeconds >= BACKWARDS_SECONDS) {
            rightEncoderSign = -rightEncoderSign;
            rightVelocity = -rightVelocity;
            rightBackwardsSeconds = 0;
            integral = 0;
        }
        if (leftBackwardsSeconds >= BACKWARDS_SECONDS) {
            leftEncoderSign = -leftEncoderSign;
            leftVelocity = -leftVelocity;
            leftBackwardsSeconds = 0;
            integral = 0;
        }

        // No reading from either wheel while both are being driven: unplugged encoder cables
        // (or both wheels jammed solid). With a speed of 0 the current cap would never let
        // the power rise, so from here the wheels are run without it.
        boolean driven = Math.abs(sentRight) > DRIVEN_POWER && Math.abs(sentLeft) > DRIVEN_POWER;
        boolean nothing = Math.abs(rightVelocity) < DEAD_ENCODER_BELOW && Math.abs(leftVelocity) < DEAD_ENCODER_BELOW;
        noReadingSeconds = driven && nothing ? noReadingSeconds + dt : 0;
        if (noReadingSeconds >= NO_READING_SECONDS) blind = true;
        // A reading came back (cable pushed in again): trust the encoders again.
        if (Math.abs(rightVelocity) > 4 * DEAD_ENCODER_BELOW || Math.abs(leftVelocity) > 4 * DEAD_ENCODER_BELOW) blind = false;
    }

    /** NaN (nothing sent yet) compares false everywhere, so that counts as "not driven". */
    private static boolean readsBackwards(double velocity, double sentPower) {
        return Math.abs(sentPower) > DRIVEN_POWER && Math.abs(velocity) > BACKWARDS_ABOVE && velocity * sentPower < 0;
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

    /** True when the target is at or above the top speed: full power, no speed control. */
    public boolean isFlatOut() {
        return Math.abs(target) >= MAX_TICKS_PER_SECOND;
    }

    private double nextPower(double measured, double dt) {
        double direction = Math.signum(target);
        boolean flatOut = isFlatOut();
        if (blind) {
            // No speed to correct with or to cap the current with. Feedforward alone gets
            // close to the target (flat out: simply full power), and easing the power in
            // slowly does the current cap's job.
            double open = flatOut ? direction
                    : Math.max(-1.0, Math.min(1.0, f * target / MAX_TICKS_PER_SECOND * battery.compensation()));
            double step = BLIND_POWER_RISE_PER_SECOND * dt;
            integral = 0;
            // Still on the way up: the drive and intake should go easy until we are there.
            if (Math.abs(open - power) > step) battery.noteFlywheelSpinUp();
            return power + Math.max(-step, Math.min(step, open - power));
        }

        double wanted;
        if (flatOut) {
            // Everything the motors have. Nothing to work out, but it still goes through the
            // current cap below on the way up.
            integral = 0;
            wanted = direction;
        } else {
            double error = target - measured;

            // Only integrate near the target. Integrating during the whole spin-up would wind
            // the term up and overshoot.
            if (Math.abs(error) < INTEGRATE_WITHIN) integral += error * dt;
            double integralPower = i / 1000.0 * integral;
            if (Math.abs(integralPower) > MAX_INTEGRAL_POWER) {
                integralPower = Math.signum(integralPower) * MAX_INTEGRAL_POWER;
                integral = integralPower * 1000.0 / Math.max(i, 1e-9);
            }

            wanted = (f * target / MAX_TICKS_PER_SECOND) * battery.compensation()
                    + p / 1000.0 * error + integralPower + feedBoost(error);
            // Never push against the direction we want to spin; over speed we just ease off.
            wanted = direction > 0 ? Math.max(0, wanted) : Math.min(0, wanted);
        }

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

    private double feedBoost(double error) {
        return feeding && target > 0 && error > FEED_BOOST_BELOW_TICKS ? FEED_BOOST_POWER : 0;
    }

    /** How many volts above the wheel's own back-EMF we allow, given the battery voltage. */
    public static double spinUpVolts(double supply) {
        if (!Double.isFinite(supply)) return SPIN_UP_VOLTS;
        if (supply >= FULL_SPIN_UP_ABOVE_VOLTS) {
            double extra = (supply - FULL_SPIN_UP_ABOVE_VOLTS) / (STRONG_BATTERY_ABOVE_VOLTS - FULL_SPIN_UP_ABOVE_VOLTS);
            return SPIN_UP_VOLTS + (STRONG_BATTERY_SPIN_UP_VOLTS - SPIN_UP_VOLTS) * Math.min(1, extra);
        }
        double fraction = (supply - GENTLE_SPIN_UP_BELOW_VOLTS) / (FULL_SPIN_UP_ABOVE_VOLTS - GENTLE_SPIN_UP_BELOW_VOLTS);
        return LOW_BATTERY_SPIN_UP_VOLTS + (SPIN_UP_VOLTS - LOW_BATTERY_SPIN_UP_VOLTS) * Math.max(0, fraction);
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
            matchBuilt = 0;
            return;
        }
        mismatch += (rightVelocity - leftVelocity - mismatch) * Math.min(1.0, dt / MATCH_SMOOTHING_SECONDS);
        // Only build once the wheels are properly turning. During the first moments of a
        // spin-up the two are naturally apart and that should not be remembered.
        if (Math.abs(speed()) > 0.3 * Math.abs(target)) {
            matchBuilt += MATCH_BUILD_GAIN / 1000.0 * mismatch * dt;
            matchBuilt = Math.max(-MAX_MATCH_POWER, Math.min(MAX_MATCH_POWER, matchBuilt));
        }
    }

    /** How far apart the two wheels are right now, in the same units as the speed. */
    public double wheelsApart() {
        return Math.abs(rightVelocity - leftVelocity);
    }

    /** aheadBy: how far this wheel is ahead of the other (ticks/s, smoothed); negative if behind. */
    private double matched(double common, double aheadBy, double built) {
        if (common == 0) return 0;
        // aheadBy is signed, so this is right for both directions: a wheel that is ahead (more
        // positive going forwards, more negative in reverse) is pulled back toward the other.
        double trim = -MATCH_GAIN / 1000.0 * 0.5 * aheadBy - built;
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

    /** Steady, at least half way there, and not above the target (that would be coasting down). */
    private boolean hasLevelledOff(double measured, long now) {
        // "At least half way there" is measured against what the wheels could ever reach, not
        // against a flat-out target of 3000 that nothing reaches: on the real robot the wheels
        // top out near 1100 at full power, and READY never came up at all.
        double halfWay = 0.5 * Math.min(target, MAX_TICKS_PER_SECOND);
        if (isFlatOut()) halfWay = 4 * DEAD_ENCODER_BELOW;
        return target > 0 && measured >= halfWay && measured <= target + READY_TOLERANCE
                && now - levelSinceNanos >= LEVEL_MS * 1e6;
    }

    /** How long the wheels have been spinning toward the present target, in seconds. 0 when stopped. */
    public double secondsSpinning() {
        return target == 0 ? 0 : (System.nanoTime() - spinStartNanos) / 1e9;
    }

    /** True right now, this loop. Use isReady() to decide when to shoot. */
    public boolean isAtSpeed() {
        return atSpeed;
    }

    /** Steady at the target, or levelled off as fast as it will go. Safe to start feeding. */
    public boolean isReady() {
        long now = System.nanoTime();
        if (blind) return target > 0 && now - spinStartNanos >= BLIND_READY_MS * 1e6;
        return (atSpeed && now - atSpeedSinceNanos >= READY_HOLD_MS * 1e6) || hasLevelledOff(speed(), now);
    }

    /**
     * For autonomous: 1 = feed, 0 = hold the index a moment longer. It only ever holds at the
     * START of a volley, until the wheels are ready, and never longer than MAX_FEED_WAIT_MS.
     * Once it has said "feed" it keeps saying it until the wheels are stopped or re-aimed.
     *
     * It used to pause between pollen as well, waiting for the wheels to come back. On the
     * real robot that left the index standing still, so it is gone: holding the speed through
     * a volley is now the wheels' job (power kept in hand, plus the feed boost), not the
     * index's. Teleop does not use this at all: there LEFT BUMPER always runs the index.
     */
    public double feedScale() {
        if (target <= 0 || feedStarted) return 1.0;
        long now = System.nanoTime();
        if (feedWaitSinceNanos == 0) feedWaitSinceNanos = now;
        if (isReady() || now - feedWaitSinceNanos >= MAX_FEED_WAIT_MS * 1e6) feedStarted = true;
        return feedStarted ? 1.0 : 0.0;
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

    /**
     * What the encoder checks found, for the Driver Station, or null when both encoders are
     * fine. The wheels keep working in every case; this is so somebody fixes the wiring.
     */
    public String encoderNote() {
        if (blind) {
            return "NO FLYWHEEL ENCODERS: running on power alone, so the speed is not held."
                    + " Check both encoder cables.";
        }
        if (rightEncoderSign < 0 && leftEncoderSign < 0) return "Both flywheel encoders count backwards (handled in code).";
        if (rightEncoderSign < 0) return "The RIGHT flywheel encoder counts backwards (handled in code).";
        if (leftEncoderSign < 0) return "The LEFT flywheel encoder counts backwards (handled in code).";
        return null;
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
