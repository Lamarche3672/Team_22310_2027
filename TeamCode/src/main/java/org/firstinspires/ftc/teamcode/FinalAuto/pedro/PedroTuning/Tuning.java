package org.firstinspires.ftc.teamcode.FinalAuto.pedro.PedroTuning;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.FinalAuto.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.FinalAuto.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.FinalAuto.pedro.procedures.Tests;
import org.firstinspires.ftc.teamcode.FinalAuto.pedro.procedures.PinpointTuner;

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

    @Tuner
    public static Procedure tests() {
        return new Tests(Constants::createDrivetrain, Constants::createLocalizer,
                Constants.FORESIGHT_TUNED ? Constants::createAlgorithm : null);
    }
}
