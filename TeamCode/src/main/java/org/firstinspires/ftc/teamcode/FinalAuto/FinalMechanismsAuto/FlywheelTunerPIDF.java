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
public class FlywheelTunerPIDF {

    /** Same as highVelocity / lowVelocity in teleop, in encoder ticks per second. */
    public static final double HIGH_VELOCITY = 1500;
    public static final double LOW_VELOCITY = 900;

    /**
     * PASTE YOUR TUNED NUMBERS HERE: the "Tuning P" and "Tuning F" the teleop tuner shows
     * once the wheels hold their speed. Leave both at 0 until you have them; the wheels then
     * use starting values worked out from the motor type (see startingCoefficients()).
     */
    public static final double P = 0;
    public static final double F = 0;

    /** Both wheels must be this close to the target speed (ticks per second) to count as "at speed". */
    public static final double READY_TOLERANCE = 75;
    /** ...and stay there this long before the first pollen is fed, so the speed has stopped swinging. */
    public static final double READY_HOLD_MS = 100;

    private DcMotorEx rightFlywheel;
    private DcMotorEx leftFlywheel;

    private double targetVelocity;
    private double rightVelocity;
    private double leftVelocity;
    private boolean atSpeed;
    private long atSpeedSinceNanos;

    public void init(HardwareMap hardwareMap) {
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "right_flywheel");
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "left_flywheel");
        for (DcMotorEx motor : new DcMotorEx[] {rightFlywheel, leftFlywheel}) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
            // Let the wheels spin down by themselves instead of slamming to a stop.
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficientsFor(motor));
        }
        stop();
    }

    private static PIDFCoefficients coefficientsFor(DcMotorEx motor) {
        if (P != 0 || F != 0) return new PIDFCoefficients(P, 0, 0, F);
        return startingCoefficients(motor.getMotorType().getAchieveableMaxTicksPerSecond());
    }

    /**
     * REV's standard starting point for velocity control when nothing has been tuned:
     * F = 32767 / (fastest the motor can spin, in ticks per second), P = F / 10, I = P / 10.
     * P = 0 and F = 0 would leave the wheels with no power at all, so that is never used.
     */
    static PIDFCoefficients startingCoefficients(double maxTicksPerSecond) {
        // 28 ticks per turn x 6000 rpm x 85 % is what almost every FTC motor reports.
        if (!(maxTicksPerSecond > 0) || Double.isInfinite(maxTicksPerSecond)) maxTicksPerSecond = 2380;
        double f = 32767.0 / maxTicksPerSecond;
        return new PIDFCoefficients(0.1 * f, 0.01 * f, 0, f);
    }

    // ------------------------------------------------------------------ direct control

    /** Start spinning to this speed (ticks per second). Returns immediately; watch isReady(). */
    public void spinUp(double velocity) {
        // Asking for the speed the wheels already have keeps them "ready".
        if (velocity != targetVelocity) atSpeed = false;
        targetVelocity = velocity;
        if (rightFlywheel != null) rightFlywheel.setVelocity(velocity);
        if (leftFlywheel != null) leftFlywheel.setVelocity(velocity);
    }

    public void stop() {
        targetVelocity = 0;
        atSpeed = false;
        // Safe to call before init() or after a failed init(): there is just nothing to stop.
        if (rightFlywheel != null) rightFlywheel.setPower(0);
        if (leftFlywheel != null) leftFlywheel.setPower(0);
    }

    /** Call once per loop: reads both wheel speeds and works out whether they are at speed. */
    public void update() {
        rightVelocity = rightFlywheel.getVelocity();
        leftVelocity = leftFlywheel.getVelocity();
        boolean inBand = targetVelocity > 0
                && Math.abs(targetVelocity - rightVelocity) <= READY_TOLERANCE
                && Math.abs(targetVelocity - leftVelocity) <= READY_TOLERANCE;
        if (inBand && !atSpeed) atSpeedSinceNanos = System.nanoTime();
        atSpeed = inBand;
    }

    /** True while both wheels are within READY_TOLERANCE of the target, right now. */
    public boolean isAtSpeed() {
        return atSpeed;
    }

    /** True once both wheels have held the target speed for READY_HOLD_MS. */
    public boolean isReady() {
        return atSpeed && (System.nanoTime() - atSpeedSinceNanos) >= READY_HOLD_MS * 1e6;
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
        telemetry.addData("Flywheel right / left", "%.0f / %.0f", rightVelocity, leftVelocity);
        telemetry.addData("Flywheel", isReady() ? "READY" : atSpeed ? "at speed" : targetVelocity > 0 ? "spinning up" : "off");
    }
}
