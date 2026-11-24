package org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.exception.TargetPositionNotSetException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FinalTeleOp.FinalMechanisms.HubReset;

public class IndexIntakeAuto {

    public static final double INTAKE_POWER = 1.0;
    public static final double INDEX_FEED_POWER = -1.0;
    public static final double INTAKE_OUTTAKE_POWER = -1.0;
    public static final double INDEX_OUTTAKE_POWER = 0.5;
    public static final double INTAKE_POWER_WHILE_FEEDING = 1.0;

    private DcMotor intakeMotor;
    private DcMotor indexMotor;
    private double intakePower;
    private double indexPower;

    public void init(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        indexMotor = hardwareMap.get(DcMotor.class, "index_motor");
        indexMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        configure();
        stop();
    }

    private void configure() {
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        indexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void intake(double power) {
        setPowers(power, 0.0);
    }

    public void intake(double power, double indexSpeed) {
        setPowers(power, INDEX_FEED_POWER * Math.max(0.0, Math.min(1.0, indexSpeed)));
    }

    public void feedShooter() {
        setPowers(INTAKE_POWER_WHILE_FEEDING, INDEX_FEED_POWER);
    }

    public void outtake() {
        setPowers(INTAKE_OUTTAKE_POWER, INDEX_OUTTAKE_POWER);
    }

    public void stop() {
        setPowers(0.0, 0.0);
    }

    private void setPowers(double intake, double index) {
        intakePower = intake;
        indexPower = index;
        if (intakeMotor == null || indexMotor == null) return;
        try {
            intakeMotor.setPower(intake);
            indexMotor.setPower(index);
        } catch (TargetPositionNotSetException hubReset) {
            HubReset.forget(intakeMotor, indexMotor);
            try {
                configure();
                intakeMotor.setPower(intake);
                indexMotor.setPower(index);
            } catch (TargetPositionNotSetException stillDown) {
            }
        }
    }

    public Command intakeCommand(double power) {
        return instant(() -> intake(power));
    }

    public Command intakeCommand(double power, double indexSpeed) {
        return instant(() -> intake(power, indexSpeed));
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
