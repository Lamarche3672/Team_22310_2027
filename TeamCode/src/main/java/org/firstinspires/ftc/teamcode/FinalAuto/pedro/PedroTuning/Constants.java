package org.firstinspires.ftc.teamcode.FinalAuto.pedro.PedroTuning;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.controllers.Controller;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public final class Constants {
    private Constants() { }

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("fl_motor");
        c.frontRightName.set("fr_motor");
        c.backLeftName.set("bl_motor");
        c.backRightName.set("br_motor");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    // ======================= PINPOINT TUNER RESULT: START =======================
    public static final boolean LOCALIZER_TUNED = true;
    public static final PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(4.29999201859829);
        c.yPodOffset.set(-6.459663841654075);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.encoderResolutionUnit.set(DistanceUnit.INCH);
        c.resetMode.set(PinpointLocalizer.ResetMode.RESET_AND_RECALIBRATE_IMU);
    });
    // ======================= PINPOINT TUNER RESULT: END =========================

    // ======================= FORESIGHT TUNER RESULT: START ======================
    public static final boolean FORESIGHT_TUNED = true;
    public static final ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.16521136745810974);
                Controller secondaryTranslationalForward = Controller.proportional(0.06104118574672952);
                Controller primaryTranslationalLateral = Controller.proportional(0.28545639379053156);
                Controller secondaryTranslationalLateral = Controller.proportional(0.105468509970281);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.009172418990242933));
                c.brake.set(Controller.proportionalFeedforward(0.0077965561417064925));

                c.headingFeedback.set(Controller.proportional(5.176801781349136));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.038024457589480504, 0.011570698151408033));

                c.linearBrakeCoefficients.set(Matrix.diag(0.05733098749031613, 0.06500723867294696));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0012834387934081333, 9.067367073473067E-4));

                c.maxAchievableForwardVelocity.set(96.18845804198172);
                c.maxAchievableStrafeVelocity.set(88.03067741871448);
                c.naturalForwardDeceleration.set(51.93510003866992);
                c.naturalStrafeDeceleration.set(72.28049587123192);
            }
    );
    // ======================= FORESIGHT TUNER RESULT: END ========================

    // Everything below builds the Pedro objects from the numbers above. Autos call create();
    // the tuners call the individual pieces.

    public static Mecanum createDrivetrain(HardwareMap map) {
        // One request per hub per loop for all encoder reads, instead of one per read.
        for (LynxModule hub : map.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        String[] names = {drivetrainConfig.frontLeftName.get(), drivetrainConfig.frontRightName.get(),
                drivetrainConfig.backLeftName.get(), drivetrainConfig.backRightName.get()};
        // Zero the motors and put them in plain power mode ourselves before Pedro touches
        // them. The last OpMode may have left them in another mode, and Pedro assumes this one.
        for (String name : names) {
            DcMotorEx motor = map.get(DcMotorEx.class, name);
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        return new Mecanum(map, drivetrainConfig);
    }

    /** The tuned Pinpoint. Keep the robot still while this runs: it recalibrates the gyro. */
    public static PinpointGuard createLocalizer(HardwareMap map) {
        if (!LOCALIZER_TUNED || !Double.isFinite(localizerConfig.xPodOffset.get())
                || !Double.isFinite(localizerConfig.yPodOffset.get()))
            throw new IllegalStateException("Pinpoint is not tuned yet. Run Pinpoint Tuner, paste its"
                    + " result over the PINPOINT TUNER RESULT block in Constants.java, then install the app again.");
        // false = a missing odometry pod is an error here. Driving a path on one pod would be wrong.
        return PinpointGuard.open(map, localizerConfig, false);
    }

    public static Foresight createAlgorithm() {
        if (!FORESIGHT_TUNED)
            throw new IllegalStateException("Driving is not tuned yet. Run Foresight Tuner, paste its"
                    + " result over the FORESIGHT TUNER RESULT block in Constants.java, then install the app again.");
        return new Foresight(foresightConfig);
    }

    /** The whole path follower, ready to drive. Throws with a plain message if tuning is not done. */
    public static Follower create(HardwareMap map) {
        Foresight algorithm = createAlgorithm();
        // Drivetrain first, so the motors are already stopped while the Pinpoint starts up.
        Mecanum drivetrain = createDrivetrain(map);
        try {
            return new Follower(createLocalizer(map), drivetrain, algorithm);
        } catch (RuntimeException error) {
            drivetrain.stop(true);
            throw error;
        }
    }
}
