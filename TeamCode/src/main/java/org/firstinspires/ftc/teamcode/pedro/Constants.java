package org.firstinspires.ftc.teamcode.pedro;

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

import java.util.OptionalDouble;

/** Pedro 3 configuration: four drive motors and two pods connected to a goBILDA Pinpoint. */
public final class Constants {
    private Constants() { }

    // Drive motor names and directions. Check them with Mecanum Tuner.
    public static final MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("fl_motor");
        c.frontRightName.set("fr_motor");
        c.backLeftName.set("bl_motor");
        c.backRightName.set("br_motor");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.manualBrakeMode.set(true);
    });

    // ======================= PINPOINT TUNER RESULT: START =======================
    // The Pinpoint Tuner gives you a block of Java. Select from the START line to
    // the END line, and paste the tuner's block over it. Nothing else to edit.
    // Until then the offsets are "not measured yet" and autonomous refuses to run.
    public static final boolean LOCALIZER_TUNED = false;
    public static final PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint"); // the Pinpoint's name in Configure Robot
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(Double.NaN); // not measured yet
        c.yPodOffset.set(Double.NaN); // not measured yet
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.encoderResolutionUnit.set(DistanceUnit.INCH);
        c.resetMode.set(PinpointLocalizer.ResetMode.RESET_AND_RECALIBRATE_IMU);
    });
    // ======================= PINPOINT TUNER RESULT: END =========================

    // ======================= FORESIGHT TUNER RESULT: START ======================
    // Same idea: paste the Foresight Tuner's block over START..END.
    // No other robot's numbers are valid for this robot.
    public static final boolean FORESIGHT_TUNED = false;
    public static final ForesightConfig foresightConfig = new ForesightConfig(c -> { });
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
