package org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Shared.Battery;
import org.firstinspires.ftc.teamcode.Shared.FlywheelControl;
import org.firstinspires.ftc.teamcode.Shared.ShotTable;

/**
 * The shooter flywheels for autonomous: the same controller teleop uses
 * (Shared/FlywheelControl), wrapped in commands an auto routine can put in a sequence.
 */
public class FlywheelAuto {
    /**
     * "Shoot at the HIGH preset": the same speed the drivers get from RIGHT BUMPER in teleop
     * when no shot is saved for the distance. An auto that passes this to spinUpCommand() or
     * volley() gets whatever HIGH is on this robot right now, including a speed the drive
     * team saved from the gamepad in shot setup. So a speed that scores in teleop from the
     * back wall is the speed the autos shoot at, with nothing to copy across by hand.
     *
     * (Autonomous used its own 2100 for a while. Teleop at 2300 then scored 8 out of 8 from
     * the back wall, and there was no reason for auto to shoot weaker than that.)
     *
     * A spot that needs its own speed can still pass any other number; only this exact
     * value is swapped for the preset.
     */
    public static final double HIGH_VELOCITY = ShotTable.DEFAULT_HIGH_PRESET;
    public static final double LOW_VELOCITY = 1800;

    private FlywheelControl control;
    private double highPreset = HIGH_VELOCITY;
    private boolean highPresetWasSaved;

    public void init(HardwareMap hardwareMap, Battery battery) {
        loadHighPreset();
        control = new FlywheelControl(hardwareMap, battery);
        stop();
    }

    /** Reads the HIGH preset the drive team saved, if they saved one. Safe without a robot. */
    public void loadHighPreset() {
        double[] saved = ShotTable.loadPresets();
        highPresetWasSaved = saved != null && saved[0] >= 400 && saved[0] <= FlywheelControl.MAX_TICKS_PER_SECOND;
        highPreset = highPresetWasSaved ? saved[0] : HIGH_VELOCITY;
    }

    /** Swaps HIGH_VELOCITY for the HIGH preset on this robot; any other speed is left as it is. */
    public double resolve(double velocity) {
        return velocity == HIGH_VELOCITY ? highPreset : velocity;
    }

    /** For the Driver Station at INIT, so the speed can be checked before the match. */
    public String describeHighPreset() {
        return String.format("%.0f (%s)", highPreset, highPresetWasSaved ? "HIGH preset saved in shot setup" : "default HIGH preset");
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
        return instant(() -> spinUp(resolve(velocity)));
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
