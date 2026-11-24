package org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
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
public class MecanumDriveAuto {

    /** "Stopped at the spot" means all four of these are true. */
    public static final double SETTLED_DISTANCE_INCHES = 1.5;
    public static final double SETTLED_HEADING_DEGREES = 3.0;
    public static final double SETTLED_SPEED_INCHES_PER_SECOND = 2.0;
    public static final double SETTLED_TURN_DEGREES_PER_SECOND = 10.0;

    /** "Blocked" means slower than both of these for STALLED_MS in a row while following a path. */
    public static final double STALLED_SPEED_INCHES_PER_SECOND = 1.0;
    public static final double STALLED_TURN_DEGREES_PER_SECOND = 5.0;
    public static final double STALLED_MS = 600;
    /** The lowest speed limit followSlowly() accepts; see the note there. */
    public static final double SLOWEST_RELIABLE_INCHES_PER_SECOND = 24;

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

    /**
     * Drives the path. Moves on when the robot reaches the end, is blocked (has not moved for
     * STALLED_MS), or after timeoutMs.
     */
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
                    // Gave up before the end (blocked, out of time, or cut off by the park deadline):
                    // stop pushing along the path and hold this spot. Pedro itself would keep
                    // driving into whatever is in the way until the next path starts.
                    if (follower.following() && !follower.atParametricEnd()) {
                        follower.hold(follower.pose());
                    }
                });
    }

    /**
     * Same as follow(), with a speed limit that belongs to this path only (Pedro removes it when
     * the path ends).
     *
     * Careful: Pedro does not measure the speed. It turns the limit into a fixed motor power of
     * about 0.009 x inchesPerSecond, so 20 is only 18 % power, too little to move a mecanum robot
     * cleanly (it creeps and stutters). Limits under SLOWEST_RELIABLE_INCHES_PER_SECOND are
     * raised to it.
     */
    public Command followSlowly(Path path, double inchesPerSecond, double timeoutMs) {
        double limit = Math.max(inchesPerSecond, SLOWEST_RELIABLE_INCHES_PER_SECOND);
        return follow(path.with(Constants.foresightConfig.maxVelocityConstraint.at(limit)), timeoutMs);
    }

    /** follow() ends just before the robot stops. Use this before shooting so the robot is steady. */
    public Command waitUntilSettled(Pose target, double timeoutMs) {
        return waitUntil(() -> isSettledAt(target)).raceWith(waitMs(timeoutMs));
    }

    public Command stopCommand() {
        return instant(this::stop);
    }

    /** Holds the robot exactly where it is now, so shooting cannot nudge it off its aim. */
    public Command holdCommand() {
        return instant(() -> follower.hold(follower.pose()));
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
