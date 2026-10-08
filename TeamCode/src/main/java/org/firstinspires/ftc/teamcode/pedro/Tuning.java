package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;
import org.firstinspires.ftc.teamcode.pedro.procedures.ThreeWheelIMUTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.ThreeWheelTuner;

public class Tuning {
    //Import @Tuners below:

    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    @Tuner
    public static Procedure tests() {
        return new Tests
                (hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig),
                        null, null);
    }

    @Tuner
    public static Procedure threeWheelTuner() {
        return new ThreeWheelTuner();
    }
// this is new and from constants. may have to move
    public static MecanumConfig driveConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("fl_motor");
                c.backLeftName.set("bl_motor");
                c.frontRightName.set("fr_motor");
                c.backRightName.set("br_motor");
                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
            }
    );

}
