package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Inputs;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.TuningOpMode;
import com.pedropathing.utils.Angle;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.PinpointGuard;

import java.util.OptionalDouble;

/**
 * Measures the Pinpoint settings by hand. No motor ever gets power here.
 *
 * <p>Three steps, each one starts by itself: keep still for a second, do the move, press Stop.
 * <ol>
 *   <li>Push FORWARD  - is the forward pod counting the right way?</li>
 *   <li>Push LEFT     - is the sideways pod counting the right way?</li>
 *   <li>Turn HALF a turn on the spot - where do the pods sit on the robot?</li>
 * </ol>
 * A single bad Pinpoint read is skipped (see {@link PinpointGuard}); it no longer fails the step.
 */
public class PinpointTuner extends Procedure {
    enum PodType {
        SWING_ARM,
        FOUR_BAR,
        CUSTOM
    }

    /** Largest believable pod distance from the robot's center, inches. */
    private static final double MAX_OFFSET_INCHES = 18.0;

    public PinpointTuner() {
        super("Pinpoint Tuner", "Measures the Pinpoint pod directions and positions. You push the robot by hand.");
    }

    @Override
    public void run() throws InterruptedException {
        Inputs inputs = inputs("Setup", "Type the Pinpoint's name from Configure Robot and pick your pod type. "
                + "Every step starts by itself: keep the robot still for one second, do the move, then press Stop.");
        Inputs.Field<String> pinpointName = inputs.s("HardwareMap Name").withDefault(Constants.localizerConfig.name.get());
        Inputs.Field<PodType> podType = inputs.e("Odometry Pod Type", PodType.class).withDefault(PodType.FOUR_BAR);
        awaitInputs(inputs);
        String name = pinpointName.get().trim();
        if (name.isEmpty()) {
            abort("Type the Pinpoint's name from Configure Robot. It is normally: pinpoint");
            return;
        }

        OptionalDouble customTicksPerInch = OptionalDouble.empty();
        if (podType.get() == PodType.CUSTOM) {
            Inputs inputsCustom = inputs("Custom pods", "How far will you push the robot forward, in inches? Measure it with a tape.");
            Inputs.Field<Double> distance = inputsCustom.d("Distance").withDefault(48.0);
            awaitInputs(inputsCustom);
            if (!Double.isFinite(distance.get()) || distance.get() <= 0.0) {
                abort("The push distance must be a number of inches above zero.");
                return;
            }
            Double ticksPerInch = finish(runOpMode(new PinpointCustomScale(name, distance.get())));
            if (ticksPerInch == null) return;
            customTicksPerInch = OptionalDouble.of(ticksPerInch);
        }

        PinpointDirectionMeasurement forward = finish(runOpMode(new PinpointDirection(name, podType.get(), customTicksPerInch, true)));
        if (forward == null) return;
        if (forward.failure() != null) {
            abort(forward.failure());
            return;
        }
        boolean xReversed = forward.reversed();

        PinpointDirectionMeasurement strafe = finish(runOpMode(new PinpointDirection(name, podType.get(), customTicksPerInch, false)));
        if (strafe == null) return;
        if (strafe.failure() != null) {
            abort(strafe.failure());
            return;
        }
        boolean yReversed = strafe.reversed();

        PinpointOffsetMeasurement offsets = finish(runOpMode(new PinpointOffsets(name, podType.get(), customTicksPerInch, xReversed, yReversed)));
        if (offsets == null) return;
        if (offsets.failure() != null) {
            abort(offsets.failure());
            return;
        }
        if (Math.abs(offsets.xPodOffset()) > MAX_OFFSET_INCHES || Math.abs(offsets.yPodOffset()) > MAX_OFFSET_INCHES) {
            abort(String.format("The pods came out %.1f and %.1f inches from the center, which is bigger than the robot. "
                    + "The robot slid while turning. Run it again and spin it on the spot.",
                    offsets.xPodOffset(), offsets.yPodOffset()));
            return;
        }

        String podTypeJava = "GoBildaPinpointDriver.GoBildaOdometryPods."
                + (podType.get() == PodType.SWING_ARM ? "goBILDA_SWINGARM_POD" : "goBILDA_4_BAR_POD");
        String xDirection = "GoBildaPinpointDriver.EncoderDirection." + (xReversed ? "REVERSED" : "FORWARD");
        String yDirection = "GoBildaPinpointDriver.EncoderDirection." + (yReversed ? "REVERSED" : "FORWARD");

        result("name", name);
        if (customTicksPerInch.isPresent()) {
            result("podType", "Custom");
            result("ticksPerUnit", customTicksPerInch.getAsDouble());
        } else {
            result("podType", podType.get());
        }
        result("xPodDirection", xReversed ? "REVERSED" : "FORWARD");
        result("yPodDirection", yReversed ? "REVERSED" : "FORWARD");
        result("xPodOffset", offsets.xPodOffset());
        result("yPodOffset", offsets.yPodOffset());

        code(Language.JAVA, "public static final boolean LOCALIZER_TUNED = true;\n" +
                "public static final PinpointConfig localizerConfig = new PinpointConfig(c -> {\n" +
                "    c.name.set(\"" + name + "\");\n" +
                (customTicksPerInch.isPresent()
                        ? "    c.ticksPerUnit.set(OptionalDouble.of(" + customTicksPerInch.getAsDouble() + "));\n"
                        : "    c.podType.set(" + podTypeJava + ");\n") +
                "    c.xPodOffset.set(" + offsets.xPodOffset() + ");\n" +
                "    c.yPodOffset.set(" + offsets.yPodOffset() + ");\n" +
                "    c.xPodDirection.set(" + xDirection + ");\n" +
                "    c.yPodDirection.set(" + yDirection + ");\n" +
                "    c.globalDistanceUnit.set(DistanceUnit.INCH);\n" +
                "    c.offsetUnits.set(DistanceUnit.INCH);\n" +
                "    c.encoderResolutionUnit.set(DistanceUnit.INCH);\n" +
                "    c.resetMode.set(PinpointLocalizer.ResetMode.RESET_AND_RECALIBRATE_IMU);\n" +
                "});");
        result("Next step", "Copy the Java code. In Constants.java paste it over everything between "
                + "PINPOINT TUNER RESULT: START and END. Install the app again, then run Tests > Pose Test.");
    }

    /** Unwraps a step's answer; aborts the procedure with a clear reason when there is none. */
    private <T> T finish(PinpointStageResult<T> result) throws InterruptedException {
        if (result == null) {
            abort("That step was stopped before the Pinpoint finished starting. Run the tuner again and "
                    + "wait for the Driver Station to say what to do before pressing Stop.");
            return null;
        }
        if (result.failure != null) {
            abort(result.failure);
            return null;
        }
        return result.value;
    }

    /** The settings every step shares. Pod offsets stay at zero until the last step measures them. */
    static PinpointConfig config(String name, PodType podType, OptionalDouble customTicksPerInch,
                                 boolean xReversed, boolean yReversed) {
        return new PinpointConfig(c -> {
            c.name.set(name);
            c.xPodDirection.set(xReversed ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD);
            c.yPodDirection.set(yReversed ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD);
            c.xPodOffset.set(0.0);
            c.yPodOffset.set(0.0);
            if (customTicksPerInch.isPresent()) {
                c.ticksPerUnit.set(OptionalDouble.of(customTicksPerInch.getAsDouble()));
            } else if (podType == PodType.SWING_ARM) {
                c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
            } else {
                c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            }
            c.globalDistanceUnit.set(DistanceUnit.INCH);
            c.offsetUnits.set(DistanceUnit.INCH);
            c.encoderResolutionUnit.set(DistanceUnit.INCH);
            c.resetMode.set(PinpointLocalizer.ResetMode.RESET_AND_RECALIBRATE_IMU);
        });
    }
}

/** What a step hands back: a value, or the reason there is none. */
final class PinpointStageResult<T> {
    final T value;
    final String failure;

    private PinpointStageResult(T value, String failure) {
        this.value = value;
        this.failure = failure;
    }

    static <T> PinpointStageResult<T> ok(T value) {
        return new PinpointStageResult<>(value, null);
    }

    static <T> PinpointStageResult<T> fail(String failure) {
        return new PinpointStageResult<>(null, failure);
    }
}

/** Shared start-up for every step: start the Pinpoint, wait until it is ready, then measure. */
abstract class PinpointStage<T> extends TuningOpMode<PinpointStageResult<T>> {
    PinpointStage(String title, String description) {
        super(title, description, true);
    }

    abstract PinpointConfig config();

    abstract T measure(PinpointGuard pinpoint) throws InterruptedException;

    @Override
    protected final PinpointStageResult<T> runTuningOpMode() throws InterruptedException {
        telemetry.addLine("KEEP THE ROBOT STILL - the Pinpoint is starting (about 1 second)");
        telemetry.update();
        try {
            // true = hand-pushed tuning: a pod warning is shown, not fatal.
            PinpointGuard pinpoint = PinpointGuard.open(hardwareMap, config(), true);
            waitForStart();
            if (isStopRequested()) return null;
            return PinpointStageResult.ok(measure(pinpoint));
        } catch (IllegalStateException problem) {
            if (isStopRequested()) return null;
            return PinpointStageResult.fail(problem.getMessage());
        }
    }

    /** One loop: read the Pinpoint. False means Stop was pressed and the step is over. */
    boolean read(PinpointGuard pinpoint) {
        if (isStopRequested()) return false;
        pinpoint.update();
        // Stop can land in the middle of a read; that read is thrown away.
        return !isStopRequested();
    }

    void show(PinpointGuard pinpoint, String whatToDo) {
        telemetry.addData("DO THIS", whatToDo);
        telemetry.addData("Pinpoint", pinpoint.lastReadGood() ? "OK" : "skipping a bad read (" + pinpoint.status() + ")");
        if (pinpoint.warning() != null) telemetry.addData("WARNING", pinpoint.warning());
    }
}

/** Custom pods only: how many encoder ticks is one inch? */
class PinpointCustomScale extends PinpointStage<Double> {
    private final String name;
    private final double distance;

    PinpointCustomScale(String name, double distance) {
        super("Custom Pod Scale",
                "Keep still for one second. Then push the robot straight FORWARD exactly " + distance
                        + " inches and press Stop.");
        this.name = name;
        this.distance = distance;
    }

    @Override
    PinpointConfig config() {
        return PinpointTuner.config(name, PinpointTuner.PodType.FOUR_BAR, OptionalDouble.empty(), false, false);
    }

    @Override
    Double measure(PinpointGuard pinpoint) {
        int start = pinpoint.ticksX();
        int travel = 0;
        while (read(pinpoint)) {
            // Int subtraction on purpose: it stays correct if the 32-bit counter wraps.
            if (pinpoint.lastReadGood()) travel = pinpoint.ticksX() - start;
            show(pinpoint, "Push FORWARD exactly " + distance + " inches, then press Stop");
            telemetry.addData("Forward pod (ticks)", travel);
            telemetry.update();
            idle();
        }
        if (travel == 0)
            throw new IllegalStateException("The forward pod did not count at all. Check the pod wheel "
                    + "touches the floor and its cable is in the Pinpoint X port.");
        return Math.abs((double) travel) / distance;
    }
}

/** Push forward (or left): which way does that pod count? */
class PinpointDirection extends PinpointStage<PinpointDirectionMeasurement> {
    private final String name;
    private final PinpointTuner.PodType podType;
    private final OptionalDouble customTicksPerInch;
    private final boolean forward;

    PinpointDirection(String name, PinpointTuner.PodType podType, OptionalDouble customTicksPerInch, boolean forward) {
        super(forward ? "Push Forward" : "Push Left",
                "Keep still for one second. Then push the robot 6 to 12 inches straight "
                        + (forward ? "FORWARD" : "to the LEFT")
                        + " without turning it, and press Stop.");
        this.name = name;
        this.podType = podType;
        this.customTicksPerInch = customTicksPerInch;
        this.forward = forward;
    }

    @Override
    PinpointConfig config() {
        return PinpointTuner.config(name, podType, customTicksPerInch, false, false);
    }

    @Override
    PinpointDirectionMeasurement measure(PinpointGuard pinpoint) {
        // goBILDA's published ticks per millimetre. Only used to show inches on screen.
        double ticksPerInch = customTicksPerInch.isPresent() ? customTicksPerInch.getAsDouble()
                : (podType == PinpointTuner.PodType.SWING_ARM ? 13.26291192 : 19.89436789) * 25.4;
        PinpointDirectionMeasurement measurement = new PinpointDirectionMeasurement(forward, ticksPerInch);
        String whatToDo = "Push " + (forward ? "FORWARD" : "LEFT") + " 6-12 inches, then press Stop";
        int startX = pinpoint.ticksX();
        int startY = pinpoint.ticksY();
        int xTicks = 0;
        int yTicks = 0;
        while (read(pinpoint)) {
            if (pinpoint.lastReadGood()) {
                // Int subtraction on purpose: it stays correct if the 32-bit counter wraps.
                xTicks = pinpoint.ticksX() - startX;
                yTicks = pinpoint.ticksY() - startY;
                measurement.sample(xTicks, yTicks);
            }
            show(pinpoint, whatToDo);
            telemetry.addData("Forward pod, X port (ticks)", xTicks);
            telemetry.addData("Sideways pod, Y port (ticks)", yTicks);
            telemetry.addData("Pushed so far (inches)", "%.1f", measurement.peakInches());
            telemetry.addData("Bad reads skipped", pinpoint.skippedReads());
            telemetry.update();
            idle();
        }
        return measurement;
    }
}

/** Spin on the spot: where do the pods sit on the robot? */
class PinpointOffsets extends PinpointStage<PinpointOffsetMeasurement> {
    private final String name;
    private final PinpointTuner.PodType podType;
    private final OptionalDouble customTicksPerInch;
    private final boolean xReversed;
    private final boolean yReversed;

    PinpointOffsets(String name, PinpointTuner.PodType podType, OptionalDouble customTicksPerInch,
                    boolean xReversed, boolean yReversed) {
        super("Turn Half a Turn",
                "Keep still for one second. Then turn the robot HALF a turn (180 degrees) counterclockwise "
                        + "on the spot, keeping its center over the same place on the floor, and press Stop. "
                        + "The Driver Station shows how far you have turned.");
        this.name = name;
        this.podType = podType;
        this.customTicksPerInch = customTicksPerInch;
        this.xReversed = xReversed;
        this.yReversed = yReversed;
    }

    @Override
    PinpointConfig config() {
        return PinpointTuner.config(name, podType, customTicksPerInch, xReversed, yReversed);
    }

    @Override
    PinpointOffsetMeasurement measure(PinpointGuard pinpoint) {
        Pose start = pinpoint.pose();
        Pose last = start;
        double previousHeading = start.heading();
        double turned = 0.0;
        while (read(pinpoint)) {
            if (pinpoint.lastReadGood()) {
                last = pinpoint.pose();
                // Add up small changes so the total keeps counting past 180 degrees.
                turned += Angle.normalizeSigned(last.heading() - previousHeading);
                previousHeading = last.heading();
            }
            show(pinpoint, "Turn HALF a turn on the spot, then press Stop");
            telemetry.addData("Turned so far (degrees)", "%.0f   (stop between %.0f and %.0f)",
                    Math.abs(Math.toDegrees(turned)),
                    PinpointOffsetMeasurement.MIN_TURN_DEGREES, PinpointOffsetMeasurement.MAX_TURN_DEGREES);
            telemetry.addData("Bad reads skipped", pinpoint.skippedReads());
            telemetry.update();
            idle();
        }
        // Express the final position in the frame the robot started in.
        double dx = last.x() - start.x();
        double dy = last.y() - start.y();
        double cos = Math.cos(-start.heading());
        double sin = Math.sin(-start.heading());
        return PinpointOffsetMeasurement.solve(dx * cos - dy * sin, dx * sin + dy * cos, turned);
    }
}
