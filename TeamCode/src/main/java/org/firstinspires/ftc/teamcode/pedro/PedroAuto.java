package org.firstinspires.ftc.teamcode.pedro;
import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Editable Pedro 3.0.1 autonomous for Team 22310.
 * Copy into TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/.
 * Requires this project's Constants.java and measured Pedro tuning.
 * Route: 24 inches forward, curve left, then return to the starting point.
 */
@Autonomous(name = "22310 Pedro Auto Example", group = "Pedro")
public final class PedroAuto extends LinearOpMode {

    // EDIT THESE POSES TO CHANGE THE ROUTE. Distances are inches.
    // X points forward from the starting position; Y points left.
    // Headings are radians: Math.toRadians(90) means 90 degrees CCW.
    private static final Pose START = new Pose(0, 0, 0);
    private static final Pose FORWARD = new Pose(24, 0, 0);
    private static final Pose CONTROL_1 = new Pose(36, 0, 0);
    private static final Pose CONTROL_2 = new Pose(36, 24, 0);
    private static final Pose CURVE_END = new Pose(24, 24, 0);

    // Target-speed limits, not a hard cap on motor power.
    private static final double MAX_PATH_SPEED = 0.25;
    private static final double MAX_VELOCITY_INCHES_PER_SECOND = 12.0;
    private static final double LEG_TIMEOUT_SECONDS = 10.0;
    private static final double AUTO_TIMEOUT_SECONDS = 25.0;
    private static final double END_POSITION_TOLERANCE_INCHES = 2.0;
    private static final double END_HEADING_TOLERANCE_DEGREES = 15.0;

    private final ElapsedTime autoTimer = new ElapsedTime();
    private Follower follower;

    // A straight path that keeps the starting heading.
    private Path forwardPath() {
        return Paths.line(START, FORWARD).linear(START, FORWARD);
    }

    // Control points shape the curve; the robot need not pass through them.
    private Path curvePath() {
        return Paths.curve(FORWARD, CONTROL_1, CONTROL_2, CURVE_END)
                .linear(FORWARD, CURVE_END);
    }

    // Return diagonally while facing the starting direction.
    private Path returnPath() {
        return Paths.line(CURVE_END, START).linear(CURVE_END, START);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        double previousPathSpeed = Constants.foresightConfig.maxPathSpeed.get();
        double previousVelocity = Constants.foresightConfig.maxVelocityConstraint.get();
        try {
            // Constants.create blocks motion until both tuning steps are complete.
            Constants.foresightConfig.maxPathSpeed.set(
                    Math.min(previousPathSpeed, MAX_PATH_SPEED));
            Constants.foresightConfig.maxVelocityConstraint.set(
                    Math.min(previousVelocity, MAX_VELOCITY_INCHES_PER_SECOND));
            follower = Constants.create(hardwareMap);
            follower.holdEnd.set(false);
            follower.setPose(START);

            telemetry.addLine("PEDRO AUTO EXAMPLE / AFTER FULL TUNING");
            telemetry.addLine("Place robot at the route origin, facing +X.");
            telemetry.addLine("Clear the route and space for the robot's full width.");
            telemetry.addLine("Route: forward -> curve left -> return.");
            telemetry.addLine("Stop, B, or right bumper aborts the routine.");
            telemetry.update();

            waitForStart();
            if (isStopRequested()) return;
            autoTimer.reset();

            if (!followLeg("1 / 3: Forward", forwardPath(), FORWARD)) return;
            // Add your scoring or mechanism action here if needed.

            if (!followLeg("2 / 3: Curve left", curvePath(), CURVE_END)) return;
            // Add another mechanism action here if needed.

            if (!followLeg("3 / 3: Return", returnPath(), START)) return;
            showStatus("Complete");
        } catch (RuntimeException error) {
            // Stop immediately before showing the error on Driver Station.
            if (follower != null) follower.stop();
            telemetry.addData("Pedro setup / runtime error", error.getMessage());
            telemetry.addLine("Check Constants.java, hardware names, and measured tuning.");
            telemetry.update();
            while (!isStopRequested()) sleep(50);
        } finally {
            try {
                if (follower != null) follower.stop();
            } finally {
                // Keep these example limits from affecting another OpMode.
                Constants.foresightConfig.maxPathSpeed.set(previousPathSpeed);
                Constants.foresightConfig.maxVelocityConstraint.set(previousVelocity);
            }
        }
    }

    /** Follow one leg; a stopped, timed-out, or missed leg ends the routine. */
    private boolean followLeg(String name, Path path, Pose target)
            throws InterruptedException {
        if (!opModeIsActive() || abortPressed()) return false;
        if (autoTimer.seconds() >= AUTO_TIMEOUT_SECONDS) {
            showStatus("Stopped: autonomous timeout");
            return false;
        }

        follower.follow(path);
        ElapsedTime legTimer = new ElapsedTime();
        while (opModeIsActive()) {
            if (abortPressed()) {
                follower.stop();
                showStatus("Aborted by driver");
                return false;
            }
            if (legTimer.seconds() >= LEG_TIMEOUT_SECONDS
                    || autoTimer.seconds() >= AUTO_TIMEOUT_SECONDS) {
                follower.stop();
                showStatus("Stopped: timeout on " + name);
                return false;
            }
            if (!follower.isBusy()) {
                follower.stop();
                Pose actual = follower.pose();
                double headingDifference = actual.heading() - target.heading();
                double headingError = Math.abs(Math.toDegrees(Math.atan2(
                        Math.sin(headingDifference), Math.cos(headingDifference))));
                if (!Double.isFinite(actual.x()) || !Double.isFinite(actual.y())
                        || !Double.isFinite(headingError)
                        || actual.distance(target) > END_POSITION_TOLERANCE_INCHES
                        || headingError > END_HEADING_TOLERANCE_DEGREES) {
                    showStatus("Stopped: endpoint missed on " + name);
                    return false;
                }
                return true;
            }

            // Pedro needs this on every loop to read odometry and drive the motors.
            follower.update();
            telemetry.addData("Path progress", "%.0f%%", follower.completion() * 100);
            showStatus(name);
            sleep(10);
        }
        follower.stop();
        return false;
    }

    private boolean abortPressed() {
        return gamepad1.b || gamepad1.right_bumper;
    }

    private void showStatus(String status) {
        Pose pose = follower.pose();
        telemetry.addData("Status", status);
        telemetry.addData("X / Y (in)", "%.1f / %.1f", pose.x(), pose.y());
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(pose.heading()));
        telemetry.addData("Run time (s)", "%.1f", autoTimer.seconds());
        telemetry.update();
    }
}
