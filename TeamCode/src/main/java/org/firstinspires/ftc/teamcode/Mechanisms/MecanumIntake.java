package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//public class MechanumIntake {

    @TeleOp(name = "Mecanum and Intake", group = "TeamCode")
    public class MecanumIntake extends OpMode {

        MecanumFieldOrientatedOpMode drive = new MecanumFieldOrientatedOpMode();
        Intake intake = new Intake();

}
