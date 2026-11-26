package org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Shared.Battery;
import org.firstinspires.ftc.teamcode.Shared.FlywheelControl;

/**
 * The shooter flywheels for autonomous: the same controller teleop uses
 * (Shared/FlywheelControl), wrapped in commands an auto routine can put in a sequence.
 */
public class FlywheelAuto {
    /**
     * Speeds in encoder ticks per second, used when there is no saved shot for the distance.
     *
     * The HD Hex tops out at 2800 on a full battery with no load. What it can HOLD, with room
     * left to recover after each pollen, drops with the battery: about 2260 at 12 V and about
     * 2060 at 11 V. 2100 is fast and still a speed the wheels can keep through a volley, so
     * every pollen leaves at the same speed. (It was 3000, then 2300: both just ran flat out.)
     * Tune it on a field.
     */
    public static final double HIGH_VELOCITY = 2100;
    public static final double LOW_VELOCITY = 1800;

    private FlywheelControl control;

    public void init(HardwareMap hardwareMap, Battery battery) {
        control = new FlywheelControl(hardwareMap, battery);
        stop();
    }

    // Every method is safe to call when init() failed (no flywheel motors in the config):
    // the auto then simply runs without shooting.

    public void spinUp(double velocity) {
        if (control != null) control.setTarget(velocity);
    }

    public void stop() {
        if (control != null) control.stop();
    }

    /** True while the index is pushing pollen in, so the wheels get their feed boost. */
    public void setFeeding(boolean feeding) {
        if (control != null) control.setFeeding(feeding);
    }

    /** AutoBase calls this every loop. */
    public void update() {
        if (control != null) control.update();
    }

    public boolean isReady() {
        return control != null && control.isReady();
    }

    /** 0 to 1: how hard the index may feed right now. 1 if there are no flywheels to wait for. */
    public double feedScale() {
        return control == null ? 1.0 : control.feedScale();
    }

    public Command spinUpCommand(double velocity) {
        return instant(() -> spinUp(velocity));
    }

    /** Waits for the wheels to be ready, but never longer than timeoutMs. */
    public Command waitUntilReady(double timeoutMs) {
        return waitUntil(this::isReady).raceWith(waitMs(timeoutMs));
    }

    public Command stopCommand() {
        return instant(this::stop);
    }

    public void addTelemetry(Telemetry telemetry) {
        if (control == null) return;
        double target = control.target();
        telemetry.addData("Flywheel target", "%.0f", target);
        if (target > FlywheelControl.HOLDABLE_TICKS_PER_SECOND) {
            telemetry.addLine(String.format("WARNING: flywheel target %.0f is above what the motors can hold (%.0f).",
                    target, FlywheelControl.HOLDABLE_TICKS_PER_SECOND));
        }
        String encoderNote = control.encoderNote();
        if (encoderNote != null) telemetry.addLine(encoderNote);
        telemetry.addData("Flywheel right / left", "%.0f / %.0f  (power %.0f%%)",
                control.rightVelocity(), control.leftVelocity(), control.power() * 100);
        telemetry.addData("Flywheel", control.isReady() ? "READY" : target > 0 ? "spinning up" : "off");
    }
}
