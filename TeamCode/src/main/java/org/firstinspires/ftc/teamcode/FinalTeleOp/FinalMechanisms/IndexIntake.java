package org.firstinspires.ftc.teamcode.FinalTeleOp.FinalMechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IndexIntake {

    private static final double INTAKE_POWER = 1.0;
    private static final double INDEX_POWER = 1.0;

    private DcMotor intakeMotor;
    private DcMotor indexMotor;

    public void init(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setPower(0);

        indexMotor = hardwareMap.get(DcMotor.class, "index_motor");
        indexMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        indexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        indexMotor.setPower(0);
    }

    public void loop(Gamepad gamepad, Telemetry telemetry) {
        double intakePower = 0.0;
        if (gamepad.right_trigger > 0.1) {
            intakePower = INTAKE_POWER;
        } else if (gamepad.left_trigger > 0.1) {
            intakePower = -INTAKE_POWER;
        }

        double indexPower = 0.0;
        if (gamepad.left_bumper) {
            indexPower = -INDEX_POWER;
        } else if (gamepad.left_trigger > 0.1) {
            indexPower = INDEX_POWER - 0.5;
        }

        intakeMotor.setPower(intakePower);
        indexMotor.setPower(indexPower);

        telemetry.addData("Intake power", intakePower);
        telemetry.addData("Index power", indexPower);
        // No telemetry.update() here: the OpMode sends everything once, after loop() ends.
        // Updating in the middle made the Driver Station flip between two half-filled screens.
    }

    public void stop() {
        // The motors are null if init() failed (for example a wrong name in the configuration).
        if (intakeMotor != null) intakeMotor.setPower(0.0);
        if (indexMotor != null) indexMotor.setPower(0.0);
    }
}