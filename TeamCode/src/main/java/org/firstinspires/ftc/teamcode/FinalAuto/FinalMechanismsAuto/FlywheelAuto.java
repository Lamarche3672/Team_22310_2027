package org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FlywheelAuto {
    public static final double HIGH_VELOCITY = 2300; // was 3000, the 40:1 HD Hex tops out at 2800
    public static final double LOW_VELOCITY = 1800;

    public static final double P = 0;
    public static final double F = 0;
    public static final double I = 0;
    public static final double READY_TOLERANCE = 75;
    public static final double READY_HOLD_MS = 100;
    public static final double LEVEL_BAND = 120;
    public static final double LEVEL_MS = 300;

    private FlywheelControl control;

    private double rightVelocity;
    private double leftVelocity;
    private boolean atSpeed;
    private long atSpeedSinceNanos;
    private double levelLowest;
    private double levelHighest;
    private long levelSinceNanos;
    private String gainSource = "";

    public void init(HardwareMap hardwareMap) {
        control = new FlywheelControl(hardwareMap, DcMotorSimple.Direction.FORWARD);
        double[] gains = chooseGains();
        control.setGains(gains[0], gains[1], gains[2]);
        gainSource = describeGainSource();
        stop();
    }

    public static double[] chooseGains() {
        if (P != 0 || F != 0) return new double[] {P, F, I};
        double[] saved = FlywheelTuning.load();
        if (saved != null) return saved;
        return new double[] {FlywheelControl.DEFAULT_P, FlywheelControl.DEFAULT_F, FlywheelControl.DEFAULT_I};
    }

    public static String describeGainSource() {
        if (P != 0 || F != 0) return "typed into FlywheelAuto";
        return FlywheelTuning.load() != null ? "saved from the teleop tuner" : "starting values (not tuned yet)";
    }

    public void spinUp(double velocity) {
        if (control == null) return;
        if (velocity != control.target()) atSpeed = false;
        control.setTarget(velocity);
    }

    public void stop() {
        atSpeed = false;
        if (control != null) control.stop();
    }

    private double target() {
        return control == null ? 0 : control.target();
    }

    public void update() {
        if (control == null) return;
        long now = System.nanoTime();
        control.update();
        rightVelocity = control.rightVelocity();
        leftVelocity = control.leftVelocity();
        double target = target();

        boolean inBand = target > 0
                && Math.abs(target - rightVelocity) <= READY_TOLERANCE
                && Math.abs(target - leftVelocity) <= READY_TOLERANCE;
        if (inBand && !atSpeed) atSpeedSinceNanos = now;
        atSpeed = inBand;

        double slower = Math.min(rightVelocity, leftVelocity);
        levelLowest = Math.min(levelLowest, slower);
        levelHighest = Math.max(levelHighest, slower);
        if (target <= 0 || levelHighest - levelLowest > LEVEL_BAND) {
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
        return target() > 0
                && Math.min(rightVelocity, leftVelocity) >= 0.5 * target()
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
        double target = target();
        telemetry.addData("Flywheel target", "%.0f", target);
        if (target > FlywheelControl.HOLDABLE_TICKS_PER_SECOND) {
            telemetry.addLine(String.format("WARNING: flywheel target %.0f is above what the motors can hold (%.0f).",
                    target, FlywheelControl.HOLDABLE_TICKS_PER_SECOND));
        }
        telemetry.addData("Flywheel right / left", "%.0f / %.0f", rightVelocity, leftVelocity);
        telemetry.addData("Flywheel power", "%.2f", control == null ? 0 : control.power());
        telemetry.addData("Flywheel", isReady() ? "READY" : atSpeed ? "at speed" : target > 0 ? "spinning up" : "off");
        telemetry.addData("Flywheel P, F, I", gainSource);
    }
}
