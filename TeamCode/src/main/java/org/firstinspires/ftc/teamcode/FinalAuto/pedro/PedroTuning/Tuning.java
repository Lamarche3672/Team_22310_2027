package org.firstinspires.ftc.teamcode.FinalAuto.pedro.PedroTuning;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.FinalAuto.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.FinalAuto.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.FinalAuto.pedro.procedures.Tests;
import org.firstinspires.ftc.teamcode.FinalAuto.pedro.procedures.PinpointTuner;

/**
 * The list of tuners that shows up in the "Pedro Tuning" OpMode on the Driver Station.
 * Pedro finds every method marked @Tuner. Only the ones that fit our robot (mecanum, two
 * pods, goBILDA Pinpoint) are listed, so nobody runs a three-wheel or OTOS tuner by mistake.
 *
 * Run them in this order: Mecanum Tuner, Pinpoint Tuner, Foresight Tuner, then Tests.
 */
public final class Tuning {
    private Tuning() { }

    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    @Tuner
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner(Constants::createLocalizer, Constants::createDrivetrain);
    }

    // The follower tests need Foresight numbers. Until those exist only the pose and
    // localization tests are offered.
    @Tuner
    public static Procedure tests() {
        return new Tests(Constants::createDrivetrain, Constants::createLocalizer,
                Constants.FORESIGHT_TUNED ? Constants::createAlgorithm : null);
    }
}
