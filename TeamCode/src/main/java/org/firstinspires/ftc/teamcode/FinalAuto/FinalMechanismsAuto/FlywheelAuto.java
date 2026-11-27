package org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Shared.Battery;
import org.firstinspires.ftc.teamcode.Shared.FlywheelControl;
import org.firstinspires.ftc.teamcode.Shared.ShooterSettings;

/**
 * The shooter flywheels for autonomous: the same controller teleop uses
 * (Shared/FlywheelControl), wrapped in commands an auto routine can put in a sequence.
 */
public class FlywheelAuto {
    /**
     * "Shoot at the set speed": the one speed the drive team dials in on gamepad 1 with the
     * D-PAD in teleop (see Shared/ShooterSettings). An auto that passes this to
     * spinUpCommand() or volley() shoots at whatever that speed is on this robot right now,
     * with the same P, I and F, so autonomous always shoots exactly like teleop does.
     *
     * A spot that needs its own speed can still pass any other number; only this exact
     * value is swapped for the set speed.
     */
    public static final double HIGH_VELOCITY = ShooterSettings.DEFAULT_SPEED;

    private FlywheelControl control;
    private ShooterSettings settings = new ShooterSettings();

    public void init(HardwareMap hardwareMap, Battery battery) {
        settings = ShooterSettings.load();
        control = new FlywheelControl(hardwareMap, battery);
        control.setGains(settings.p, settings.f, settings.i);
        stop();
    }

    /** Swaps HIGH_VELOCITY for the set speed on this robot; any other speed is left as it is. */
    public double resolve(double velocity) {
        return velocity == HIGH_VELOCITY ? settings.speed : velocity;
    }

    /** For the Driver Station at INIT, so the numbers can be checked before the match. */
    public String describeSettings() {
        return settings.describe() + "  (set in teleop with the D-PAD)";
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

    public Command spinUpCommand(double velocity) {
        return instant(() -> spinUp(resolve(velocity)));
    }

    /**
     * Waits until the wheels have been spinning for at least this long (never longer than
     * that again, in case they were never started). Wheels that were started earlier, on the
     * drive over, have usually had their time already and this returns at once.
     */
    public Command waitUntilSpunFor(double milliseconds) {
        return waitUntil(() -> control == null || control.secondsSpinning() * 1000 >= milliseconds)
                .raceWith(waitMs(milliseconds));
    }

    public Command stopCommand() {
        return instant(this::stop);
    }

    public void addTelemetry(Telemetry telemetry) {
        if (control == null) return;
        double target = control.target();
        telemetry.addData("Flywheel target", "%.0f", target);
        String encoderNote = control.encoderNote();
        if (encoderNote != null) telemetry.addLine(encoderNote);
        telemetry.addData("Flywheel right / left", "%.0f / %.0f  (%.0f apart, power %.0f%%)",
                control.rightVelocity(), control.leftVelocity(), control.wheelsApart(), control.power() * 100);
        telemetry.addData("Flywheel", control.isReady() ? "READY" : target > 0 ? "spinning up" : "off");
    }
}
