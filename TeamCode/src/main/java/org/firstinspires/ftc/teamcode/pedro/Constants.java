package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.Encoder;
import com.pedropathing.revhub.localizers.ThreeWheelConfig;
import com.pedropathing.revhub.localizers.ThreeWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** Pedro 3 configuration for the team's four-motor X-drive and three passive pods. */
public final class Constants {
    private Constants() { }

    // Pedro's wheel mix has the same signs as DriveMath.wheels when turn is CCW.
    // These motor names, ports, and polarities match the V3 hardware mapping.
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

    // After the Pedro Three Wheel Tuner, replace this block with its Java output.
    // The placeholder geometry deliberately cannot be used for localization.
    public static final boolean LOCALIZER_TUNED = false;
    public static final ThreeWheelConfig localizerConfig = new ThreeWheelConfig(c -> {
        c.leftEncoderName.set("br.motor");   // port 1, left parallel pod
        c.rightEncoderName.set("fl.motor");  // port 2, right parallel pod
        c.strafeEncoderName.set("bl.motor"); // port 0, sideways pod
        c.leftPodY.set(Double.NaN);
        c.rightPodY.set(Double.NaN);
        c.strafePodX.set(Double.NaN);
        c.forwardTicksToInches.set(Double.NaN);
        c.strafeTicksToInches.set(Double.NaN);
        c.turnTicksToRadians.set(Double.NaN);
        c.leftEncoderDirection.set(Encoder.FORWARD);
        c.rightEncoderDirection.set(Encoder.FORWARD);
        c.strafeEncoderDirection.set(Encoder.FORWARD);
    });

    // After the Foresight Tuner, replace this block with its Java output.
    // No sample robot's motion constants are valid for this robot.
    public static final boolean FORESIGHT_TUNED = false;
    public static final ForesightConfig foresightConfig = new ForesightConfig(c -> { });

    public static Mecanum createDrivetrain(HardwareMap map) {
        String[] names = {"fl.motor", "fr.motor", "bl.motor", "br.motor"};
        int[] ports = {2, 3, 0, 1};
        DcMotorEx first = null;
        for (int i = 0; i < names.length; i++) {
            DcMotorEx motor = map.get(DcMotorEx.class, names[i]);
            motor.setPower(0);
            if (motor.getPortNumber() != ports[i] || first != null && motor.getController() != first.getController())
                throw new IllegalStateException("Pedro motor mapping differs from V3: " + names[i]);
            if (first == null) first = motor;
        }
        return new Mecanum(map, drivetrainConfig);
    }

    public static ThreeWheelLocalizer createLocalizer(HardwareMap map) {
        if (!LOCALIZER_TUNED || !Double.isFinite(localizerConfig.leftPodY.get())
                || !Double.isFinite(localizerConfig.rightPodY.get())
                || !Double.isFinite(localizerConfig.strafePodX.get())
                || !Double.isFinite(localizerConfig.forwardTicksToInches.get())
                || !Double.isFinite(localizerConfig.strafeTicksToInches.get())
                || !Double.isFinite(localizerConfig.turnTicksToRadians.get())
                || !(localizerConfig.forwardTicksToInches.get() > 0)
                || !(localizerConfig.strafeTicksToInches.get() > 0)
                || !(localizerConfig.turnTicksToRadians.get() > 0)
                || !(localizerConfig.leftPodY.get() > 0)
                || !(localizerConfig.rightPodY.get() < 0)
                || !"br.motor".equals(localizerConfig.leftEncoderName.get())
                || !"fl.motor".equals(localizerConfig.rightEncoderName.get())
                || !"bl.motor".equals(localizerConfig.strafeEncoderName.get()))
            throw new IllegalStateException("Run Pedro Three Wheel Tuner and enter its measured config in Constants.java");
        return new ThreeWheelLocalizer(map, localizerConfig);
    }

    public static Follower create(HardwareMap map) {
        if (!FORESIGHT_TUNED)
            throw new IllegalStateException("Run Pedro Foresight Tuner and enter its measured config in Constants.java");
        // Construct the drivetrain first so motor direction is set before pod encoders are read.
        Mecanum drivetrain = createDrivetrain(map);
        try {
            return new Follower(createLocalizer(map), drivetrain, new Foresight(foresightConfig));
        } catch (RuntimeException error) {
            drivetrain.stop(true);
            throw error;
        }
    }
}


