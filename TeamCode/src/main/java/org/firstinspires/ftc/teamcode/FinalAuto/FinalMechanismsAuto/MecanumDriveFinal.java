package org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FinalAuto.pedro.PedroTuning.Constants;

/**
 * Autonomous version of FinalTeleOp/FinalMechanisms/MecanumDriveFinal.
 *
 * In teleop the sticks drive the four wheels. In autonomous Pedro's Follower drives them
 * along paths, using the same four motors and the same Pinpoint (set up in Constants.java).
 * This class is the one place the routine talks to the drivetrain:
 *
 *   follow(path, timeout)                 drive a path at full speed
 *   followSlowly(path, speed, timeout)    drive a path with a speed limit (for intaking)
 *   waitUntilSettled(pose, timeout)       wait until the robot has really stopped at a spot
 *
 * Every wait has a timeout, and a drive also ends early when the robot is blocked (not
 * moving at all for STALLED_MS), so a robot stuck against a wall moves on instead of freezing.
 */
public class MecanumDriveFinal {

    /** "Stopped at the spot" means all four of these are true. */
    public static final double SETTLED_DISTANCE_INCHES = 1.5;
    public static final double SETTLED_HEADING_DEGREES = 3.0;
    public static final double SETTLED_SPEED_INCHES_PER_SECOND = 2.0;
    public static final double SETTLED_TURN_DEGREES_PER_SECOND = 10.0;

    /** "Blocked" means slower than both of these for STALLED_MS in a row while following a path. */
    public static final double STALLED_SPEED_INCHES_PER_SECOND = 1.0;
    public static final double STALLED_TURN_DEGREES_PER_SECOND = 5.0;
    public static final double STALLED_MS = 600;

    private Follower follower;

    /** Keep the robot still while this runs: the Pinpoint calibrates for about one second. */
    public void init(HardwareMap hardwareMap, Pose startPose) {
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
    }

    /** Call once per loop: reads the Pinpoint and powers the wheels. */
    public void update() {
        follower.update();
    }

    public Pose pose() {
        return follower.pose();
    }

    public Follower follower() {
        return follower;
    }

    /** Lets go of the wheels and removes any speed limit a path set. Safe to call any time. */
    public void stop() {
        if (follower != null) follower.stop();
    }

    // ------------------------------------------------------------------ commands for a routine

    /** Drives the path. Moves on when the robot reaches the end, is blocked, or after timeoutMs. */
    public Command follow(Path path, double timeoutMs) {
        return PedroCommands.follow(follower, path).raceWith(waitMs(timeoutMs), waitUntilStalled());
    }

    /** Finishes once the robot has not moved for STALLED_MS, counted from when the drive starts. */
    private Command waitUntilStalled() {
        long[] lastMovedNanos = new long[1];
        return new CommandBuilder()
                .setStart(() -> lastMovedNanos[0] = System.nanoTime())
                .setDone(() -> {
                    long now = System.nanoTime();
                    if (isMoving()) lastMovedNanos[0] = now;
                    return now - lastMovedNanos[0] >= STALLED_MS * 1e6;
                });
    }

    /**
     * Same as follow(), but never faster than inchesPerSecond.
     * The limit belongs to this path only; Pedro removes it when the path ends.
     */
    public Command followSlowly(Path path, double inchesPerSecond, double timeoutMs) {
        return follow(path.with(Constants.foresightConfig.maxVelocityConstraint.at(inchesPerSecond)), timeoutMs);
    }

    /** follow() ends just before the robot stops. Use this before shooting so the robot is steady. */
    public Command waitUntilSettled(Pose target, double timeoutMs) {
        return waitUntil(() -> isSettledAt(target)).raceWith(waitMs(timeoutMs));
    }

    public Command stopCommand() {
        return instant(this::stop);
    }

    public boolean isMoving() {
        Velocity velocity = follower.velocity();
        return Math.hypot(velocity.vx, velocity.vy) > STALLED_SPEED_INCHES_PER_SECOND
                || Math.abs(velocity.omega) > Math.toRadians(STALLED_TURN_DEGREES_PER_SECOND);
    }

    /** True when the robot's center is within this many inches of the spot. */
    public boolean isNear(Pose target, double inches) {
        return follower.pose().distance(target) <= inches;
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
    }
}
