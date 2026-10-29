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

    // Pedro's wheel mix has the same signs as DriveMath.wheels when turn is CCW.
    // Keep the existing drive motor names and polarities; verify with Mecanum Tuner.
    public static final MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("fl.motor");
        c.frontRightName.set("fr.motor");
        c.backLeftName.set("bl.motor");
        c.backRightName.set("br.motor");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.manualBrakeMode.set(true);
    });

    // After the Pinpoint Tuner, replace localizerConfig with its Java output,
    // then set LOCALIZER_TUNED to true. Do NOT guess the measured values.
    // Both pods plug into the Pinpoint, NOT the drive-motor encoder sockets.
    // The Pinpoint has its own IMU; no REV Hub IMU name/orientation is needed.
    public static final boolean LOCALIZER_TUNED = false;
    public static final PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint"); // goBILDA Pinpoint I2C device name in Robot Configuration
        // Provisional pod type; select your actual pods in the Pinpoint Tuner.
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(Double.NaN);
        c.yPodOffset.set(Double.NaN);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.encoderResolutionUnit.set(DistanceUnit.INCH);
        c.resetMode.set(PinpointLocalizer.ResetMode.RESET_AND_RECALIBRATE_IMU);
    });

    // After the Foresight Tuner, replace this block with its Java output.
    // No sample robot's motion constants are valid for this robot.
    public static final boolean FORESIGHT_TUNED = false;
    public static final ForesightConfig foresightConfig = new ForesightConfig(c -> { });

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

    public static PinpointLocalizer createLocalizer(HardwareMap map) {
        if (!LOCALIZER_TUNED || !Double.isFinite(localizerConfig.xPodOffset.get())
                || !Double.isFinite(localizerConfig.yPodOffset.get())
                || localizerConfig.name.get().trim().isEmpty()
                || localizerConfig.xPodDirection.get() == null
                || localizerConfig.yPodDirection.get() == null
                || localizerConfig.globalDistanceUnit.get() != DistanceUnit.INCH
                || (localizerConfig.ticksPerUnit.get().isPresent()
                && (!Double.isFinite(localizerConfig.ticksPerUnit.get().getAsDouble())
                || localizerConfig.ticksPerUnit.get().getAsDouble() <= 0.0)))
            throw new IllegalStateException("Run Pinpoint Tuner, paste its config, then set LOCALIZER_TUNED = true in Constants.java");
        return initializePinpoint(map, localizerConfig);
    }

    /** Shared initialization for the calibrated localizer and manual Pinpoint tuner. */
    public static PinpointLocalizer initializePinpoint(HardwareMap map, PinpointConfig config) {
        // Stop the drivetrain before the Pinpoint calibrates. Keep the robot still.
        createDrivetrain(map).stop(false);
        PinpointLocalizer localizer = new PinpointLocalizer(map, config);
        GoBildaPinpointDriver pinpoint = map.get(GoBildaPinpointDriver.class, config.name.get());
        if (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY)
            throw new IllegalStateException("Pinpoint not ready: " + pinpoint.getDeviceStatus()
                    + ". Check both pods, I2C connection, and keep still during calibration.");
        return localizer;
    }

    public static Foresight createAlgorithm() {
        if (!FORESIGHT_TUNED)
            throw new IllegalStateException("Run Foresight Tuner, paste its config, then set FORESIGHT_TUNED = true in Constants.java");
        return new Foresight(foresightConfig);
    }

    public static Follower create(HardwareMap map) {
        Foresight algorithm = createAlgorithm();
        // Stop the motors before initializing/calibrating the Pinpoint.
        Mecanum drivetrain = createDrivetrain(map);
        try {
            return new Follower(createLocalizer(map), drivetrain, algorithm);
        } catch (RuntimeException error) {
            drivetrain.stop(true);
            throw error;
        }
    }
}
