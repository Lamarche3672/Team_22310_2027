package org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Autonomous version of FinalTeleOp/FinalMechanisms/FlywheelTunerPIDF.
 *
 * Same two motors, same direction, same velocity control as teleop. The difference:
 * there is no gamepad, so the speed is set by spinUp() and the P and F numbers are
 * fixed below instead of being tuned live with the d-pad.
 *
 * Call update() once per loop. isReady() then says when the wheels are fast enough to shoot.
 */
public class FlywheelAuto {
    public static final double HIGH_VELOCITY = 3000;
    public static final double LOW_VELOCITY = 1800;

    public static final double P = 0;
    public static final double F = 0;
    public static final double READY_TOLERANCE = 75;
    public static final double READY_HOLD_MS = 100;
    /**
     * Second way to be "ready": the wheels have stopped speeding up. If the target is faster than
     * the motors can go (or the tuning leaves them a little short) they never get within
     * READY_TOLERANCE, and every volley used to wait out the whole spin-up timeout. Now a speed
     * whose highest and lowest reading over LEVEL_MS are no more than LEVEL_BAND apart, above
     * half the target, also counts. (The band has to be wider than the normal ripple in the
     * speed reading, or the wheels would never look level.)
     */
    public static final double LEVEL_BAND = 120;
    public static final double LEVEL_MS = 300;
    /**
     * The wheels are walked up to their speed over about this long instead of being thrown to
     * full power in one step. Two motors starting from rest at full power is a big hit on the battery.
     */
    public static final double SPIN_UP_SECONDS = 0.4;
    /** While spinning, the speed is sent again this often, so the wheels come back after a hub brown-out. */
    private static final long RESEND_NANOS = 250_000_000L;
    /** While the wheels are off their speed is only read this often, to keep the drive loop fast. */
    private static final long IDLE_READ_NANOS = 200_000_000L;

    private DcMotorEx rightFlywheel;
    private DcMotorEx leftFlywheel;

    private double targetVelocity;
    private double rightVelocity;
    private double leftVelocity;
    private boolean atSpeed;
    private long atSpeedSinceNanos;
    private double levelLowest;
    private double levelHighest;
    private long levelSinceNanos;
    private double commandedVelocity;
    private long lastSendNanos;
    private long lastUpdateNanos;
    private long lastReadNanos;
    private String coefficientSource = "";
    /** Fastest speed the configured motor type says it can hold, in ticks per second. */
    private double reachableVelocity = Double.POSITIVE_INFINITY;

    public void init(HardwareMap hardwareMap) {
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "right_flywheel");
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "left_flywheel");
        for (DcMotorEx motor : new DcMotorEx[] {rightFlywheel, leftFlywheel}) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
            // Let the wheels spin down by themselves instead of slamming to a stop.
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, chooseCoefficients(motor));
            double max = motor.getMotorType().getAchieveableMaxTicksPerSecond();
            if (max > 0) reachableVelocity = Math.min(reachableVelocity, max);
        }
        coefficientSource = describeCoefficientSource();
        stop();
    }

    /**
     * Which P and F the wheels get, in this order:
     *   1. FlywheelAuto.P and F above, when they are typed in.
     *   2. The numbers saved from the teleop tuner (press A there), see FlywheelTuning.
     *   3. REV's starting values for this motor, so the wheels always spin.
     * MAIN TELEOP starts from the same numbers.
     */
    public static PIDFCoefficients chooseCoefficients(DcMotorEx motor) {
        if (P != 0 || F != 0) return new PIDFCoefficients(P, 0, 0, F);
        double[] saved = FlywheelTuning.load();
        if (saved != null) return new PIDFCoefficients(saved[0], saved[2], 0, saved[1]);
        return startingCoefficients(motor.getMotorType().getAchieveableMaxTicksPerSecond());
    }

    public static String describeCoefficientSource() {
        if (P != 0 || F != 0) return "typed into FlywheelAuto";
        return FlywheelTuning.load() != null ? "saved from the teleop tuner" : "starting values (not tuned yet)";
    }

    /**
     * A starting point for velocity control when nothing has been tuned, from REV's guide:
     * F = 32767 / (fastest the motor can spin, in ticks per second). REV suggests P = F / 10 and
     * I = P / 10; this robot uses a stronger P = F / 2 and I = F / 100.
     * P = 0 and F = 0 would leave the wheels with no power at all, so that is never used.
     */
    public static PIDFCoefficients startingCoefficients(double maxTicksPerSecond) {
        // 28 ticks per turn x 6000 rpm x 85 % is what almost every FTC motor reports.
        if (!(maxTicksPerSecond > 0) || Double.isInfinite(maxTicksPerSecond)) maxTicksPerSecond = 2380;
        double f = 32767.0 / maxTicksPerSecond;
        return new PIDFCoefficients(0.5 * f, 0.01 * f, 0, f);
    }

    // ------------------------------------------------------------------ direct control

    /**
     * Start spinning to this speed (ticks per second). Returns immediately; update(), called every
     * loop, walks the wheels up to it and isReady() says when they are there.
     */
    public void spinUp(double velocity) {
        // Asking for the speed the wheels already have keeps them "ready".
        if (velocity != targetVelocity) atSpeed = false;
        targetVelocity = velocity;
    }

    public void stop() {
        targetVelocity = 0;
        commandedVelocity = 0;
        atSpeed = false;
        // Safe to call before init() or after a failed init(): there is just nothing to stop.
        for (DcMotorEx motor : new DcMotorEx[] {rightFlywheel, leftFlywheel}) {
            if (motor == null) continue;
            // setPower(0) can be skipped by the SDK when it was the last power it remembers, which
            // would leave the wheels spinning. These two are always sent: target 0, then motor off
            // (coasting). setVelocity() in spinUp() switches the motor back on by itself.
            motor.setVelocity(0);
            motor.setMotorDisable();
        }
    }

    /**
     * Call once per loop. Walks the wheels toward the wanted speed, reads both wheel speeds and
     * works out whether they are at speed.
     */
    public void update() {
        long now = System.nanoTime();
        double dt = lastUpdateNanos == 0 ? 0.0 : Math.min(0.1, (now - lastUpdateNanos) / 1e9);
        lastUpdateNanos = now;

        if (targetVelocity > 0) {
            double maxStep = dt * targetVelocity / SPIN_UP_SECONDS;
            double next = commandedVelocity
                    + Math.max(-maxStep, Math.min(maxStep, targetVelocity - commandedVelocity));
            if (next != commandedVelocity || now - lastSendNanos >= RESEND_NANOS) {
                rightFlywheel.setVelocity(next);
                leftFlywheel.setVelocity(next);
                lastSendNanos = now;
            }
            commandedVelocity = next;
        }

        // Reading the speed is a request to the hub. While the wheels are off it is only for show,
        // and the loop is better spent on driving.
        if (targetVelocity > 0 || now - lastReadNanos >= IDLE_READ_NANOS) {
            rightVelocity = rightFlywheel.getVelocity();
            leftVelocity = leftFlywheel.getVelocity();
            lastReadNanos = now;
        }

        boolean inBand = targetVelocity > 0
                && Math.abs(targetVelocity - rightVelocity) <= READY_TOLERANCE
                && Math.abs(targetVelocity - leftVelocity) <= READY_TOLERANCE;
        if (inBand && !atSpeed) atSpeedSinceNanos = now;
        atSpeed = inBand;

        // Levelling off: follow the lowest and highest reading of the slower wheel, and restart
        // the clock from the newest reading whenever the two drift further apart than the band.
        double slower = Math.min(rightVelocity, leftVelocity);
        levelLowest = Math.min(levelLowest, slower);
        levelHighest = Math.max(levelHighest, slower);
        if (targetVelocity <= 0 || levelHighest - levelLowest > LEVEL_BAND) {
            levelLowest = slower;
            levelHighest = slower;
            levelSinceNanos = now;
        }
    }

    /** Speed of the slower of the two wheels, in ticks per second. */
    public double slowerWheelSpeed() {
        return Math.min(rightVelocity, leftVelocity);
    }

    /** True while both wheels are within READY_TOLERANCE of the target, right now. */
    public boolean isAtSpeed() {
        return atSpeed;
    }

    /** True once both wheels have held the target speed for READY_HOLD_MS, or have levelled off. */
    public boolean isReady() {
        long now = System.nanoTime();
        return (atSpeed && now - atSpeedSinceNanos >= READY_HOLD_MS * 1e6) || hasLevelledOff(now);
    }

    private boolean hasLevelledOff(long now) {
        return targetVelocity > 0
                && Math.min(rightVelocity, leftVelocity) >= 0.5 * targetVelocity
                && now - levelSinceNanos >= LEVEL_MS * 1e6;
    }

    // ------------------------------------------------------------------ commands for a routine

    /** Starts the wheels and moves straight to the next step, so the robot can drive meanwhile. */
    public Command spinUpCommand(double velocity) {
        return instant(() -> spinUp(velocity));
    }

    /** Waits for the wheels to be ready. Gives up after timeoutMs so the routine can never hang here. */
    public Command waitUntilReady(double timeoutMs) {
        return waitUntil(this::isReady).raceWith(waitMs(timeoutMs));
    }

    public Command stopCommand() {
        return instant(this::stop);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Flywheel target", "%.0f", targetVelocity);
        if (targetVelocity > reachableVelocity) {
            telemetry.addLine(String.format("WARNING: flywheel target %.0f is above what this motor type can hold (%.0f)."
                    + " The wheels run at full power and never become READY.", targetVelocity, reachableVelocity));
        }
        telemetry.addData("Flywheel right / left", "%.0f / %.0f", rightVelocity, leftVelocity);
        telemetry.addData("Flywheel", isReady() ? "READY" : atSpeed ? "at speed" : targetVelocity > 0 ? "spinning up" : "off");
        telemetry.addData("Flywheel P and F", coefficientSource);
    }
}
