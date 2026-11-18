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

/** Pedro 3 configuration: four drive motors and two pods connected to a goBILDA Pinpoint. */
public final class Constants {
    private Constants() { }

    // Drive motor names and directions. Check them with Mecanum Tuner.
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
    // The Pinpoint Tuner gives you a block of Java. Select from the START line to
    // the END line, and paste the tuner's block over it. Nothing else to edit.
    // Until then the offsets are "not measured yet" and autonomous refuses to run.
    public static final boolean LOCALIZER_TUNED = true;
    public static final PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        c.xPodOffset.set(5.508228706414574);
        c.yPodOffset.set(-12.357994176674342);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.encoderResolutionUnit.set(DistanceUnit.INCH);
        c.resetMode.set(PinpointLocalizer.ResetMode.RESET_AND_RECALIBRATE_IMU);
    });
    // ======================= PINPOINT TUNER RESULT: END =========================

    // ======================= FORESIGHT TUNER RESULT: START ======================
    // Same idea: paste the Foresight Tuner's block over START..END.
    // No other robot's numbers are valid for this robot.
    public static final boolean FORESIGHT_TUNED = true;
    public static final ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.20535422765262534);
                Controller secondaryTranslationalForward = Controller.proportional(0.07587289995162362);
                Controller primaryTranslationalLateral = Controller.proportional(0.4899651926313773);
                Controller secondaryTranslationalLateral = Controller.proportional(0.18102904656622595);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.009580578794314563));
                c.brake.set(Controller.proportionalFeedforward(0.008143491975167378));

                c.headingFeedback.set(Controller.proportional(6.359810674477428));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.039229888274301364, 0.010511051527930236));

                c.linearBrakeCoefficients.set(Matrix.diag(0.08222615406263734, 0.020477282947824794));
                c.quadraticBrakeCoefficients.set(Matrix.diag(5.714933804491514E-4, 0.0014979838538168005));

                c.maxAchievableForwardVelocity.set(90.61170634831896);
                c.maxAchievableStrafeVelocity.set(73.10512269571213);
                c.naturalForwardDeceleration.set(55.8730971963151);
                c.naturalStrafeDeceleration.set(63.44716781893875);
            }
    );
    // ======================= FORESIGHT TUNER RESULT: END ========================

    public static Mecanum createDrivetrain(HardwareMap map) {
        for (LynxModule hub : map.getAll(LynxModule.class)) {
            // A previous OpMode may have left MANUAL caching enabled.
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        String[] names = {drivetrainConfig.frontLeftName.get(), drivetrainConfig.frontRightName.get(),
                drivetrainConfig.backLeftName.get(), drivetrainConfig.backRightName.get()};
        for (String name : names) {
            DcMotorEx motor = map.get(DcMotorEx.class, name);
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        return new Mecanum(map, drivetrainConfig);
    }

    /**
     * The tuned Pinpoint, ready to use. Keep the robot still while this runs (about 1 s).
     * A single bad read is skipped, not fatal; see PinpointGuard.
     */
    public static PinpointGuard createLocalizer(HardwareMap map) {
        if (!LOCALIZER_TUNED || !Double.isFinite(localizerConfig.xPodOffset.get())
                || !Double.isFinite(localizerConfig.yPodOffset.get()))
            throw new IllegalStateException("Pinpoint is not tuned yet. Run Pinpoint Tuner, paste its"
                    + " result over the PINPOINT TUNER RESULT block in Constants.java, then install the app again.");
        // false = this localizer is used to drive the robot, so a missing pod is an error.
        return PinpointGuard.open(map, localizerConfig, false);
    }

    public static Foresight createAlgorithm() {
        if (!FORESIGHT_TUNED)
            throw new IllegalStateException("Driving is not tuned yet. Run Foresight Tuner, paste its"
                    + " result over the FORESIGHT TUNER RESULT block in Constants.java, then install the app again.");
        return new Foresight(foresightConfig);
    }

    public static Follower create(HardwareMap map) {
        Foresight algorithm = createAlgorithm();
        // Create the drivetrain first so the motors are stopped while the Pinpoint calibrates.
        Mecanum drivetrain = createDrivetrain(map);
        try {
            return new Follower(createLocalizer(map), drivetrain, algorithm);
        } catch (RuntimeException error) {
            drivetrain.stop(true);
            throw error;
        }
    }
}
