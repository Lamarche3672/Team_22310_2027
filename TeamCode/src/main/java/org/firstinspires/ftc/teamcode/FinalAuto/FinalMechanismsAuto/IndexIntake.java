package org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Autonomous version of FinalTeleOp/FinalMechanisms/IndexIntake.
 *
 * Same two motors, same directions and same power signs as teleop.
 * Each gamepad button became a method:
 *
 *   RIGHT TRIGGER  (intake)              ->  intake(power)
 *   LEFT BUMPER    (index inwards)       ->  feedShooter()
 *   LEFT TRIGGER   (outtake everything)  ->  outtake()
 *   nothing held                         ->  stop()
 *
 * The ...Command() methods wrap those so an autonomous routine can put them in a sequence.
 */
public class IndexIntake {

    /** Right trigger in teleop: pull pollen in. */
    public static final double INTAKE_POWER = 1.0;
    /** Left bumper in teleop: index inwards, pushing pollen up into the flywheels. */
    public static final double INDEX_FEED_POWER = -1.0;
    /** Left trigger in teleop: spit everything back out. */
    public static final double INTAKE_OUTTAKE_POWER = -1.0;
    public static final double INDEX_OUTTAKE_POWER = 0.5;
    /**
     * While feeding the flywheels the intake also turns inwards, so a pollen still sitting in
     * the intake gets pushed up to the index. Set to 0 to feed with the index motor only.
     */
    public static final double INTAKE_POWER_WHILE_FEEDING = 1.0;

    private DcMotor intakeMotor;
    private DcMotor indexMotor;
    private double intakePower;
    private double indexPower;

    public void init(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        indexMotor = hardwareMap.get(DcMotor.class, "index_motor");
        indexMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        indexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        stop();
    }

    // ------------------------------------------------------------------ direct control

    /** Pull pollen in. The index stays off, exactly like holding only the right trigger. */
    public void intake(double power) {
        setPowers(power, 0.0);
    }

    /** Push the stored pollen up into the flywheels. Spin the flywheels up first. */
    public void feedShooter() {
        setPowers(INTAKE_POWER_WHILE_FEEDING, INDEX_FEED_POWER);
    }

    /** Spit everything back out the front. */
    public void outtake() {
        setPowers(INTAKE_OUTTAKE_POWER, INDEX_OUTTAKE_POWER);
    }

    public void stop() {
        setPowers(0.0, 0.0);
    }

    private void setPowers(double intake, double index) {
        intakePower = intake;
        indexPower = index;
        // Safe to call before init() or after a failed init(): there is just nothing to stop.
        if (intakeMotor != null) intakeMotor.setPower(intake);
        if (indexMotor != null) indexMotor.setPower(index);
    }

    // ------------------------------------------------------------------ commands for a routine

    /** Turns the intake on and moves straight to the next step. It keeps running until stopped. */
    public Command intakeCommand(double power) {
        return instant(() -> intake(power));
    }

    public Command outtakeCommand() {
        return instant(this::outtake);
    }

    public Command stopCommand() {
        return instant(this::stop);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Intake / index power", "%.2f / %.2f", intakePower, indexPower);
    }
}
