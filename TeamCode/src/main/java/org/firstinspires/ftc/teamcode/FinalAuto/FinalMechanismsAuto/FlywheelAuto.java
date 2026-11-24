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

public class FlywheelAuto {
    public static final double HIGH_VELOCITY = 3000;
    public static final double LOW_VELOCITY = 1800;

    public static final double P = 0;
    public static final double F = 0;
    public static final double READY_TOLERANCE = 75;
    public static final double READY_HOLD_MS = 100;
    public static final double LEVEL_BAND = 120;
    public static final double LEVEL_MS = 300;
    public static final double SPIN_UP_SECONDS = 0.4;
    private static final long RESEND_NANOS = 250_000_000L;
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
    private double reachableVelocity = Double.POSITIVE_INFINITY;

    public void init(HardwareMap hardwareMap) {
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "right_flywheel");
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "left_flywheel");
        for (DcMotorEx motor : new DcMotorEx[] {rightFlywheel, leftFlywheel}) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, chooseCoefficients(motor));
            double max = motor.getMotorType().getAchieveableMaxTicksPerSecond();
            if (max > 0) reachableVelocity = Math.min(reachableVelocity, max);
        }
        coefficientSource = describeCoefficientSource();
        stop();
    }

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

    public static PIDFCoefficients startingCoefficients(double maxTicksPerSecond) {
        if (!(maxTicksPerSecond > 0) || Double.isInfinite(maxTicksPerSecond)) maxTicksPerSecond = 2380;
        double f = 32767.0 / maxTicksPerSecond;
        return new PIDFCoefficients(0.5 * f, 0.01 * f, 0, f);
    }

    public void spinUp(double velocity) {
        if (velocity != targetVelocity) atSpeed = false;
        targetVelocity = velocity;
    }

    public void stop() {
        targetVelocity = 0;
        commandedVelocity = 0;
        atSpeed = false;
        for (DcMotorEx motor : new DcMotorEx[] {rightFlywheel, leftFlywheel}) {
            if (motor == null) continue;
            motor.setVelocity(0);
            motor.setMotorDisable();
        }
    }

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

        double slower = Math.min(rightVelocity, leftVelocity);
        levelLowest = Math.min(levelLowest, slower);
        levelHighest = Math.max(levelHighest, slower);
        if (targetVelocity <= 0 || levelHighest - levelLowest > LEVEL_BAND) {
            levelLowest = slower;
            levelHighest = slower;
            levelSinceNanos = now;
        }
    }

    public double slowerWheelSpeed() {
        return Math.min(rightVelocity, leftVelocity);
    }

    public boolean isAtSpeed() {
        return atSpeed;
    }

    public boolean isReady() {
        long now = System.nanoTime();
        return (atSpeed && now - atSpeedSinceNanos >= READY_HOLD_MS * 1e6) || hasLevelledOff(now);
    }

    private boolean hasLevelledOff(long now) {
        return targetVelocity > 0
                && Math.min(rightVelocity, leftVelocity) >= 0.5 * targetVelocity
                && now - levelSinceNanos >= LEVEL_MS * 1e6;
    }

    public Command spinUpCommand(double velocity) {
        return instant(() -> spinUp(velocity));
    }

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
