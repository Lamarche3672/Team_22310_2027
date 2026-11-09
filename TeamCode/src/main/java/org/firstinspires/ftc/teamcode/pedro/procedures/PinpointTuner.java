package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Inputs;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.TuningOpMode;
import com.pedropathing.utils.Angle;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.*;

public class PinpointTuner extends Procedure {
    enum PodType {
        SWING_ARM,
        FOUR_BAR,
        CUSTOM
    }
    public PinpointTuner() {
        super("Pinpoint Tuner", "A procedure for tuning the Pinpoint localizer.");
    }

    @Override
    public void run() throws InterruptedException {
        Inputs inputs = inputs("Setup", "Set the goBILDA Pinpoint I2C device name and actual pod type. Keep the robot STILL during each stage's initialization/calibration.");
        Inputs.Field<String> pinpointName = inputs.s("HardwareMap Name").withDefault(Constants.localizerConfig.name.get());
        Inputs.Field<PodType> podType = inputs.e("Odometry Pod Type", PodType.class).withDefault(PodType.FOUR_BAR);
        awaitInputs(inputs);
        if (pinpointName.get().trim().isEmpty()) {
            abort("Enter the Pinpoint I2C device name from Robot Configuration, normally pinpoint.");
            return;
        }
        String name = pinpointName.get().trim();

        OptionalDouble customPodScalar = OptionalDouble.empty();

        if (podType.get() == PodType.CUSTOM) {
            Inputs inputsCustom = inputs("Custom Scalar Identification Push Distance", "Set the distance you will push your robot forward in inches");
            Inputs.Field<Double> distance = inputsCustom.d("Distance").withDefault(48.0);
            awaitInputs(inputsCustom);
            if (!Double.isFinite(distance.get()) || distance.get() <= 0.0) {
                abort("Enter a finite positive push distance in inches.");
                return;
            }
            Double scalar = runOpMode(new PinpointCustomPodScalar(distance.get(), name));
            if (scalar == null || !Double.isFinite(scalar) || scalar <= 0.0) {
                abort("No valid forward encoder measurement. Check pod connections, complete the push, then press Stop.");
                return;
            }
            customPodScalar = OptionalDouble.of(scalar);
        }

        PinpointDirectionMeasurement forward = runOpMode(new PinpointForwardDirection(name, podType.get(), customPodScalar));
        if (forward == null || forward.failure() != null) {
            abort(forward == null ? "Forward test stopped before Start. Start the test, push FORWARD, then Stop." : forward.failure());
            return;
        }
        boolean forwardPodReversed = forward.reversed();
        PinpointDirectionMeasurement strafe = runOpMode(new PinpointStrafeDirection(name, podType.get(), customPodScalar));
        if (strafe == null || strafe.failure() != null) {
            abort(strafe == null ? "Strafe test stopped before Start. Start the test, push LEFT, then Stop." : strafe.failure());
            return;
        }
        boolean strafePodReversed = strafe.reversed();

        List<Double> offsets = runOpMode(new PinpointOffsets(name, podType.get(), customPodScalar, forwardPodReversed, strafePodReversed));
        if (offsets == null || !Double.isFinite(offsets.get(0)) || !Double.isFinite(offsets.get(1))) {
            abort("Invalid offset measurement. Keep the center fixed and rotate exactly 180 degrees CCW after Start, then Stop.");
            return;
        }

        result("name", name);

        if (customPodScalar.isPresent()) {
            result("podType", "Custom");
            result("ticksPerUnit", customPodScalar.getAsDouble());
        } else {
            result("podType", podType.get() == PodType.SWING_ARM ? GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD : GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        }

        result("xPodDirection", forwardPodReversed ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD);
        result("yPodDirection", strafePodReversed ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD);
        result("xPodOffset", offsets.get(0));
        result("yPodOffset", offsets.get(1));

        code(Language.JAVA,"public static PinpointConfig localizerConfig = new PinpointConfig(c -> {\n" +
                "    c.name.set(\"" + name + "\");\n" +
                (customPodScalar.isPresent() ? "    c.ticksPerUnit.set(OptionalDouble.of(" + customPodScalar.getAsDouble() + "));\n" : "    c.podType.set(" + (podType.get() == PodType.SWING_ARM ? "GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD" : "GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD") + ");\n") +
                "    c.xPodOffset.set(" + offsets.get(0) + ");\n" +
                "    c.yPodOffset.set(" + offsets.get(1) + ");\n" +
                "    c.xPodDirection.set(" + (forwardPodReversed ? "GoBildaPinpointDriver.EncoderDirection.REVERSED" : "GoBildaPinpointDriver.EncoderDirection.FORWARD") + ");\n" +
                "    c.yPodDirection.set(" + (strafePodReversed ? "GoBildaPinpointDriver.EncoderDirection.REVERSED" : "GoBildaPinpointDriver.EncoderDirection.FORWARD") + ");\n" +
                "    c.globalDistanceUnit.set(DistanceUnit.INCH);\n" +
                "    c.offsetUnits.set(DistanceUnit.INCH);\n" +
                "    c.encoderResolutionUnit.set(DistanceUnit.INCH);\n" +
                "    c.resetMode.set(PinpointLocalizer.ResetMode.RESET_AND_RECALIBRATE_IMU);\n" +
                "});");
        result("Next step", "Paste the Java localizerConfig into Constants.java, set LOCALIZER_TUNED = true, and rebuild before Foresight tuning.");
    }

    static PinpointLocalizer localizer(HardwareMap map, PinpointConfig config) {
        // No motor power is applied by the manual Pinpoint tuner.
        config.globalDistanceUnit.set(DistanceUnit.INCH);
        config.offsetUnits.set(DistanceUnit.INCH);
        config.encoderResolutionUnit.set(DistanceUnit.INCH);
        config.resetMode.set(PinpointLocalizer.ResetMode.RESET_AND_RECALIBRATE_IMU);
        return Constants.initializePinpoint(map, config);
    }
}

class PinpointCustomPodScalar extends TuningOpMode<Double> {

    String name;
    double distance;

    public PinpointCustomPodScalar(Double distance, String name) {
        super("Custom Scalar Identification",
                "Determines the scalar for the custom pods of the Pinpoint localizer. \n"
                        + "Push your robot forward " + distance + " inches exactly and then stop the Opmode",
                true);
        this.name = name;
        this.distance = distance;
    }

    @Override
    protected Double runTuningOpMode() throws InterruptedException {
        PinpointConfig config = new PinpointConfig(c -> {
            c.name.set(name);
            c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            c.xPodOffset.set(0.0);
            c.yPodOffset.set(0.0);
        });
        PinpointTuner.localizer(hardwareMap, config);
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, name);
        Thread.sleep(1000);
        waitForStart();
        if (isStopRequested()) return null;
        pinpoint.update();
        int startTicks = pinpoint.getEncoderX();
        int travelTicks = 0;
        while (!isStopRequested()) {
            pinpoint.update();
            if (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY)
                return null;
            // Use actual counts, not pose inches with a fake one-tick/inch scale.
            // That fake scale can trip the SDK driver's position corruption checks.
            travelTicks = pinpoint.getEncoderX() - startTicks;
            telemetry.addData("Forward travel (raw ticks)", travelTicks);
            telemetry.update();
            idle();
        }
        return Math.abs((double) travelTicks) / distance;
    }
}

class PinpointDirection extends TuningOpMode<PinpointDirectionMeasurement> {
    String name;
    PinpointTuner.PodType podType;
    OptionalDouble customPodScalar;
    final boolean forward;

    PinpointDirection(String name, PinpointTuner.PodType podType, OptionalDouble customPodScalar, boolean forward) {
        super(forward ? "Forward Direction Identification" : "Strafe Direction Identification",
                "Keep still during initialization. Press Start, THEN push 6-12 inches "
                        + (forward ? "FORWARD" : "LEFT")
                        + " in a straight line without turning. Watch the raw ticks, then press Stop.",
                true);
        this.name = name;
        this.podType = podType;
        this.customPodScalar = customPodScalar;
        this.forward = forward;
    }

    @Override
    protected PinpointDirectionMeasurement runTuningOpMode() throws InterruptedException {
        PinpointConfig config = new PinpointConfig(c -> {
            c.name.set(name);
            c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            c.xPodOffset.set(0.0);
            c.yPodOffset.set(0.0);
            if (customPodScalar.isPresent()) {
                c.encoderResolutionUnit.set(DistanceUnit.INCH);
                c.ticksPerUnit.set(OptionalDouble.of(customPodScalar.getAsDouble()));
            } else {
                c.podType.set(podType == PinpointTuner.PodType.SWING_ARM ? GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD : GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            }
        });
        PinpointTuner.localizer(hardwareMap, config);
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, name);
        // These are the FTC SDK's manufacturer resolutions, not measured robot offsets.
        double ticksPerInch = customPodScalar.isPresent() ? customPodScalar.getAsDouble()
                : (podType == PinpointTuner.PodType.SWING_ARM ? 13.26291192 : 19.89436789) * 25.4;
        PinpointDirectionMeasurement measurement = new PinpointDirectionMeasurement(forward, ticksPerInch);
        telemetry.addData("Next", "Press Start, then push %s 6-12 inches", forward ? "FORWARD" : "LEFT");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return null;
        GoBildaPinpointDriver.DeviceStatus initialStatus = readStatus(pinpoint);
        if (initialStatus != GoBildaPinpointDriver.DeviceStatus.READY) {
            measurement.fail("Pinpoint is " + initialStatus + ". Keep still and retry; check pods and I2C wiring.");
            return measurement;
        }
        int startX = pinpoint.getEncoderX();
        int startY = pinpoint.getEncoderY();
        while (!isStopRequested()) {
            GoBildaPinpointDriver.DeviceStatus status = readStatus(pinpoint);
            if (status != GoBildaPinpointDriver.DeviceStatus.READY) {
                measurement.fail("Pinpoint reported " + status + ". Measurement rejected; check pods and I2C wiring, then retry.");
                return measurement;
            }
            // Int subtraction intentionally handles a 32-bit counter wrap over a short push.
            int xTicks = pinpoint.getEncoderX() - startX;
            int yTicks = pinpoint.getEncoderY() - startY;
            measurement.sample(xTicks, yTicks);
            telemetry.addData("Pinpoint status", status);
            telemetry.addData("X encoder travel (ticks)", xTicks);
            telemetry.addData("Y encoder travel (ticks)", yTicks);
            telemetry.addData("Largest valid-axis travel (in)", measurement.peakInches());
            telemetry.addData("Action", "Push %s 6-12 inches, then Stop", forward ? "FORWARD" : "LEFT");
            telemetry.update();
            idle();
        }
        return measurement;
    }

    private static GoBildaPinpointDriver.DeviceStatus readStatus(GoBildaPinpointDriver pinpoint)
            throws InterruptedException {
        // Retry transient corrupt reads, but never use their cached encoder values.
        long deadline = System.nanoTime() + 500_000_000L;
        while (true) {
            if (Thread.currentThread().isInterrupted()) throw new InterruptedException();
            pinpoint.update();
            GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
            if (status != GoBildaPinpointDriver.DeviceStatus.FAULT_BAD_READ
                    || System.nanoTime() >= deadline) return status;
            Thread.sleep(20);
        }
    }
}

class PinpointForwardDirection extends PinpointDirection {
    PinpointForwardDirection(String name, PinpointTuner.PodType podType, OptionalDouble customPodScalar) {
        super(name, podType, customPodScalar, true);
    }
}

class PinpointStrafeDirection extends PinpointDirection {
    public PinpointStrafeDirection(String name, PinpointTuner.PodType podType, OptionalDouble customPodScalar) {
        super(name, podType, customPodScalar, false);
    }
}

class PinpointOffsets extends TuningOpMode<List<Double>> {
    String name;
    PinpointTuner.PodType podType;
    OptionalDouble customPodScalar =  OptionalDouble.empty();
    boolean forwardPodReversed, strafePodReversed;
    private Pose previous = Pose.zero();

    public PinpointOffsets(String name, PinpointTuner.PodType podType, OptionalDouble customPodScalar, Boolean forwardPodReversed, Boolean strafePodReversed) {
        super("Offsets Identification",
                "Automatically identifies the offsets for your Pinpoint localizer. \n"
                        + "After Start, rotate exactly 180 degrees counterclockwise about a FIXED center, then Stop. Keep still during initialization.",
                true);
        this.name = name;
        this.podType = podType;
        if (customPodScalar.isPresent()) {
            this.customPodScalar = customPodScalar;
        }
        this.forwardPodReversed = forwardPodReversed;
        this.strafePodReversed = strafePodReversed;
    }

    @Override
    protected List<Double> runTuningOpMode() throws InterruptedException {
        PinpointConfig config = new PinpointConfig(c -> {
            c.name.set(name);
            c.xPodDirection.set(forwardPodReversed ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD);
            c.yPodDirection.set(strafePodReversed ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD);
            if (customPodScalar.isPresent()) {
                c.encoderResolutionUnit.set(DistanceUnit.INCH);
                c.ticksPerUnit.set(OptionalDouble.of(customPodScalar.getAsDouble()));
                c.resetMode.set(PinpointLocalizer.ResetMode.RESET_AND_RECALIBRATE_IMU);
            } else {
                c.podType.set(podType == PinpointTuner.PodType.SWING_ARM ? GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD : GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            }
            c.xPodOffset.set(0.0);
            c.yPodOffset.set(0.0);
            c.globalDistanceUnit.set(DistanceUnit.INCH);
            c.offsetUnits.set(DistanceUnit.INCH);
        });
        PinpointLocalizer localizer = PinpointTuner.localizer(hardwareMap, config);
        localizer.setPose(Pose.zero());
        localizer.update();


        Thread.sleep(1000);
        waitForStart();
        localizer.setPose(Pose.zero());
        localizer.update();

        double previousHeading = localizer.pose().heading();
        double totalHeading = 0.0;

        while (!isStopRequested()) {
            previous = localizer.pose();
            localizer.update();
            double heading = localizer.pose().heading();
            totalHeading += Angle.normalizeSigned(heading - previousHeading);
            previousHeading = heading;

            telemetry.addData("CCW turn (degrees)", Math.toDegrees(totalHeading));
            telemetry.addData("pose", localizer.pose());
            telemetry.addData("previous", previous);
            telemetry.update();
        }

        // The endpoint formula below assumes a half-turn, not an arbitrary spin.
        if (!Double.isFinite(totalHeading) || Math.abs(totalHeading - Math.PI) > Math.toRadians(5)) {
            return null;
        }

        if (localizer.pose().x() != Pose.zero().x() || localizer.pose().y() != Pose.zero().y()) {
            previous =  localizer.pose();
        }

        return List.of(((-previous.y()) / 2.0), ((-previous.x()) / 2.0));
    }
}
