package org.firstinspires.ftc.teamcode.Mechanisms;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

//@TeleOp(name = "Intake", group = "TeamCode")
public class Intake{

    static double maxPower = 0.6;
    static double maxSpeed = 0.6;
    private static DcMotor intakeMotor;

    public void init(HardwareMap hwMap) {
        intakeMotor = hwMap.get(DcMotor.class, "intake_motor");
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

        public static void loop (double forward, double backward){
            while (gamepad1.right_bumper) {
                System.out.println(intakeMotor);
                intakeMotor.setPower(maxSpeed * (forward / maxPower));
            }

            while (gamepad1.left_bumper) {
                System.out.println(intakeMotor);
                intakeMotor.setPower(maxSpeed * (backward / maxPower));
            }
        }

}

