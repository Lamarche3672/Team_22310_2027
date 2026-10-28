package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;

/** Only register the procedures needed by this two-pod + Pinpoint robot. */
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
        // Pose/driving/localization tests work before Foresight tuning.
        // Follower tests remain unavailable until the algorithm has been tuned.
        return new Tests(Constants::createDrivetrain, Constants::createLocalizer,
                Constants.FORESIGHT_TUNED ? Constants::createAlgorithm : null);
    }
}
