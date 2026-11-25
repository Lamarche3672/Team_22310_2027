package org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.exception.TargetPositionNotSetException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FinalAuto.pedro.PedroTuning.Constants;
import org.firstinspires.ftc.teamcode.FinalAuto.pedro.PedroTuning.PinpointGuard;
import org.firstinspires.ftc.teamcode.Shared.HiveAim;
import org.firstinspires.ftc.teamcode.Shared.HubReset;

/**
 * The drive for autonomous: Pedro Pathing's Follower, wrapped in commands that cannot hang.
 *
 * Pedro on its own will keep trying to finish a path forever. In a match that means a robot
 * pinned against a wall with the motors pushing until the buzzer. Every drive command here
 * ends on one of three things: it arrived, it ran out of time, or it stopped moving.
 */
public class MecanumDriveAuto {

    // "Settled" = on the spot and nearly at rest. Used before shooting and when parking.
    public static final double SETTLED_DISTANCE_INCHES = 1.5;
    public static final double SETTLED_HEADING_DEGREES = 3.0;
    public static final double SETTLED_SPEED_INCHES_PER_SECOND = 2.0;
    public static final double SETTLED_TURN_DEGREES_PER_SECOND = 10.0;

    // "Stalled" = meant to be driving but barely moving for STALLED_MS: blocked by a wall or
    // another robot. The drive gives up and the routine moves on.
    public static final double STALLED_SPEED_INCHES_PER_SECOND = 1.0;
    public static final double STALLED_TURN_DEGREES_PER_SECOND = 5.0;
    public static final double STALLED_MS = 600;

    /**
     * Pedro turns a speed limit into a fixed motor power (about 0.009 x inches per second).
     * Under 24 in/s that is below what a mecanum drive needs to move at all, so slower limits
     * are bumped up to this.
     */
    public static final double SLOWEST_RELIABLE_INCHES_PER_SECOND = 24;

    /** After this many bad Pinpoint reads in a row the position is stale: stop driving on it. */
    public static final int BLIND_READS = 3;

    // Camera aim. A reading that wants a turn bigger than this is more likely the wrong tag
    // (or a reflection) than a real error, so it is ignored.
    public static final double MAX_AIM_CORRECTION_DEGREES = 25;
    public static final double AIMED_DEGREES = 1.5;
    public static final double AIMED_TURN_DEGREES_PER_SECOND = 20;
    /** No hive in view after this long: stop looking and shoot on the route heading. */
    public static final double AIM_GIVE_UP_MS = 400;

    private Follower follower;
    private DcMotorEx[] driveMotors = new DcMotorEx[0];
    private String aimNote = "";

    public void init(HardwareMap hardwareMap, Pose startPose) {
        follower = Constants.create(hardwareMap);
        driveMotors = new DcMotorEx[] {
                hardwareMap.get(DcMotorEx.class, Constants.drivetrainConfig.frontLeftName.get()),
                hardwareMap.get(DcMotorEx.class, Constants.drivetrainConfig.frontRightName.get()),
                hardwareMap.get(DcMotorEx.class, Constants.drivetrainConfig.backLeftName.get()),
                hardwareMap.get(DcMotorEx.class, Constants.drivetrainConfig.backRightName.get())};
        follower.setPose(startPose);
        follower.update();
    }

    /** Call every loop: reads the Pinpoint and drives the motors. */
    public void update() {
        try {
            step();
        } catch (TargetPositionNotSetException hubReset) {
            recoverFromHubReset();
        }
    }

    private void step() {
        // While the Pinpoint is not answering, the last known position just sits there, and
        // Pedro would keep driving on it at full speed. Brake instead, and keep reading; if it
        // does not come back within a second PinpointGuard throws and the auto ends.
        Localizer localizer = follower.localizer;
        if (localizer instanceof PinpointGuard && ((PinpointGuard) localizer).badStreak() >= BLIND_READS) {
            localizer.update();
            follower.drivetrain.stop(true);
            return;
        }
        follower.update();
    }

    private void recoverFromHubReset() {
        // The hub restarted and forgot the motor setup. Send it again.
        HubReset.forget(driveMotors);
        try {
            for (DcMotorEx motor : driveMotors) {
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            // Pedro remembers the last power it sent each wheel and skips "the same" value.
            // The refused write is in that memory, so zero everything through Pedro: the
            // next update() then really sends the powers again.
            if (follower != null) follower.drivetrain.stop(true);
        } catch (TargetPositionNotSetException stillDown) {
            // Still coming back. Tried again on the next refusal.
        }
    }

    public Pose pose() {
        return follower.pose();
    }

    public Follower follower() {
        return follower;
    }

    public void stop() {
        if (follower == null) return;
        try {
            // follower.stop() only changes Pedro's mode; the motors are zeroed explicitly
            // because nothing else will run after this at the end of an OpMode.
            follower.stop();
            follower.drivetrain.stop();
        } catch (TargetPositionNotSetException hubReset) {
            recoverFromHubReset();
            for (DcMotorEx motor : driveMotors) {
                try {
                    motor.setPower(0);
                } catch (TargetPositionNotSetException stillDown) {
                    // The SDK zeroes everything itself when the OpMode ends.
                }
            }
        }
    }

    // ------------------------------------------------------------------ commands

    /** Follows a path. Ends when it arrives, when timeoutMs is up, or when the robot is stuck. */
    public Command follow(Path path, double timeoutMs) {
        long[] startedNanos = new long[1];
        long[] lastMovedNanos = new long[1];
        return new CommandBuilder()
                .setStart(() -> {
                    follower.follow(path);
                    startedNanos[0] = System.nanoTime();
                    lastMovedNanos[0] = startedNanos[0];
                })
                .setDone(() -> {
                    long now = System.nanoTime();
                    if (isMoving()) lastMovedNanos[0] = now;
                    return follower.atParametricEnd()
                            || now - startedNanos[0] >= timeoutMs * 1e6
                            || now - lastMovedNanos[0] >= STALLED_MS * 1e6;
                })
                .setEnd(reason -> {
                    // Gave up part way: hold where we are, so the motors stop pushing at
                    // whatever is in the way.
                    if (follower.following() && !follower.atParametricEnd()) {
                        follower.hold(follower.pose());
                    }
                });
    }

    /** Like follow(), with a top speed. Used while intaking so pollen are not bulldozed. */
    public Command followSlowly(Path path, double inchesPerSecond, double timeoutMs) {
        double limit = Math.max(inchesPerSecond, SLOWEST_RELIABLE_INCHES_PER_SECOND);
        return follow(path.with(Constants.foresightConfig.maxVelocityConstraint.at(limit)), timeoutMs);
    }

    public Command waitUntilSettled(Pose target, double timeoutMs) {
        return waitUntil(() -> isSettledAt(target)).raceWith(waitMs(timeoutMs));
    }

    public Command stopCommand() {
        return instant(this::stop);
    }

    /** Actively hold the current spot (so a bump while shooting is pushed back against). */
    public Command holdCommand() {
        return instant(() -> follower.hold(follower.pose()));
    }

    /**
     * Turns on the spot to face the hive, using the camera's aim heading, and stays held
     * there. If the camera has nothing (no Limelight, hive not in view, or a reading we do
     * not believe) the robot keeps the heading the route gave it and this ends quickly.
     */
    public Command aimAtHive(HiveAim aim, double timeoutMs) {
        if (aim == null) return instant(() -> aimNote = "no camera");
        long[] startedNanos = new long[1];
        Pose[] spot = new Pose[1];
        double[] target = {Double.NaN};
        return new CommandBuilder()
                .setStart(() -> {
                    startedNanos[0] = System.nanoTime();
                    spot[0] = follower.pose();
                    target[0] = Double.NaN;
                    aimNote = "looking for the hive";
                })
                .setExecute(() -> {
                    if (!aim.hasAim()) return;
                    double wanted = aim.aimHeading();
                    double correction = Angle.normalizeSigned(wanted - spot[0].heading());
                    if (Math.abs(correction) > Math.toRadians(MAX_AIM_CORRECTION_DEGREES)) {
                        aimNote = String.format("camera wanted %.0f deg, too much - ignored", Math.toDegrees(correction));
                        return;
                    }
                    // Only re-aim when the camera's answer has really moved; re-issuing the
                    // hold every loop would keep restarting Pedro's turn.
                    if (Double.isNaN(target[0]) || Math.abs(Angle.normalizeSigned(wanted - target[0])) > Math.toRadians(0.5)) {
                        target[0] = wanted;
                        follower.hold(spot[0].withHeading(wanted));
                        aimNote = String.format("camera aim, turned %.1f deg", Math.toDegrees(correction));
                    }
                })
                .setDone(() -> {
                    double elapsedMs = (System.nanoTime() - startedNanos[0]) / 1e6;
                    if (elapsedMs >= timeoutMs) return true;
                    if (Double.isNaN(target[0])) {
                        if (elapsedMs < AIM_GIVE_UP_MS) return false;
                        if (aimNote.startsWith("looking")) aimNote = "hive not in view, shot on route heading";
                        return true;
                    }
                    double off = Math.abs(Angle.normalizeSigned(target[0] - follower.pose().heading()));
                    return off <= Math.toRadians(AIMED_DEGREES)
                            && Math.abs(follower.velocity().omega) <= Math.toRadians(AIMED_TURN_DEGREES_PER_SECOND);
                });
    }

    // ------------------------------------------------------------------ questions

    public boolean isMoving() {
        Velocity velocity = follower.velocity();
        return Math.hypot(velocity.vx, velocity.vy) > STALLED_SPEED_INCHES_PER_SECOND
                || Math.abs(velocity.omega) > Math.toRadians(STALLED_TURN_DEGREES_PER_SECOND);
    }

    public boolean isNear(Pose target, double inches) {
        return follower.pose().distance(target) <= inches;
    }

    public boolean isFacing(Pose target, double degrees) {
        return Math.abs(Angle.normalizeSigned(target.heading() - follower.pose().heading()))
                <= Math.toRadians(degrees);
    }

    public boolean isSettledAt(Pose target) {
        Pose pose = follower.pose();
        Velocity velocity = follower.velocity();
        return pose.distance(target) <= SETTLED_DISTANCE_INCHES
                && Math.abs(Angle.normalizeSigned(target.heading() - pose.heading())) <= Math.toRadians(SETTLED_HEADING_DEGREES)
                && Math.hypot(velocity.vx, velocity.vy) <= SETTLED_SPEED_INCHES_PER_SECOND
                && Math.abs(velocity.omega) <= Math.toRadians(SETTLED_TURN_DEGREES_PER_SECOND);
    }

    public void addTelemetry(Telemetry telemetry) {
        Pose pose = follower.pose();
        telemetry.addData("x / y (in)", "%.1f / %.1f", pose.x(), pose.y());
        telemetry.addData("heading (deg)", "%.1f", Math.toDegrees(pose.heading()));
        if (follower.currentPath() != null) {
            telemetry.addData("Distance left on path (in)", "%.1f", follower.distanceToEndpoint());
        }
        if (!aimNote.isEmpty()) telemetry.addData("Last aim", aimNote);
    }
}
