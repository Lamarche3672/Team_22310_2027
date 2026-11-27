package org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Shared.Battery;
import org.firstinspires.ftc.teamcode.Shared.IntakeAndIndex;

/**
 * The intake and index belt for autonomous: the same motors as teleop
 * (Shared/IntakeAndIndex), wrapped in commands an auto routine can put in a sequence.
 */
public class IndexIntakeAuto {
    private IntakeAndIndex motors;

    public void init(HardwareMap hardwareMap, Battery battery) {
        motors = new IntakeAndIndex(hardwareMap, battery);
    }

    /** AutoBase calls this every loop so the power ramps keep moving. */
    public void update() {
        if (motors != null) motors.update();
    }

    /** Intake on, with the belt creeping (indexSpeed 0 to 1) so pollen make room for the next. */
    public void intake(double indexSpeed) {
        if (motors != null) motors.intake(indexSpeed);
    }

    /**
     * Push pollen into the flywheels. scale 0 to 1, see FlywheelControl.feedScale().
     *
     * Only the index runs, the same as LEFT BUMPER in teleop. The intake used to run here too,
     * but measured on the robot the flywheels pull about 10 A and the index 6 A, all through
     * the one hub, and every extra amp there costs flywheel speed at the exact moment a
     * pollen arrives. The intake's 3 A was load for nothing: the belt feeds on its own.
     */
    public void feedShooter(double scale) {
        if (motors != null) motors.command(0.0, scale);
    }

    public void outtake() {
        if (motors != null) motors.outtake();
    }

    public void stop() {
        if (motors != null) motors.stop();
    }

    public Command intakeCommand(double indexSpeed) {
        return instant(() -> intake(indexSpeed));
    }

    public Command outtakeCommand() {
        return instant(this::outtake);
    }

    public Command stopCommand() {
        return instant(this::stop);
    }

    public void addTelemetry(Telemetry telemetry) {
        if (motors == null) return;
        telemetry.addData("Intake / index power", "%.2f / %.2f", motors.intakePower(), motors.indexPower());
    }
}
