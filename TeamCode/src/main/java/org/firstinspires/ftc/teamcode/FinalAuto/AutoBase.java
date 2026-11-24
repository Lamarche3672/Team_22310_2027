package org.firstinspires.ftc.teamcode.FinalAuto;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.lazy;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto.FlywheelAuto;
import org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto.IndexIntakeAuto;
import org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto.MecanumDriveAuto;

/**
 * Everything the scoring autonomous routines share, so each routine file only has to say
 * WHAT the robot does:
 *
 *   - the three mechanisms (drive, intake, flywheel), started and always stopped again
 *   - the loop that runs the routine and shows it on the Driver Station
 *   - PARK at the end, also when the routine is cut short by the park deadline
 *   - the blue alliance: the same routine with every pose turned half a turn around the field
 *
 * A routine extends this and fills in: autoName(), startPose(), parkPose(), scoringRoutine().
 */
public abstract class AutoBase extends LinearOpMode {

    protected static final double SETTLE_TIMEOUT_MS = 1500;
    protected static final double PARK_TIMEOUT_MS = 4000;
    /** Autonomous is 30 s. At this point the robot stops scoring and goes to park. */
    protected static final double PARK_DEADLINE_MS = 25000;
    /** Closer than this to the parking spot there is nothing left to drive. */
    private static final double PARK_ALREADY_THERE_INCHES = 1;
    private static final double FIELD_MIDDLE_INCHES = 72;

    protected final MecanumDriveAuto drive = new MecanumDriveAuto();
    protected final IndexIntakeAuto intake = new IndexIntakeAuto();
    protected final FlywheelAuto flywheel = new FlywheelAuto();

    protected final ElapsedTime autoTimer = new ElapsedTime();
    protected String currentStep = "Waiting for start";

    // ------------------------------------------------------------------ what a routine fills in

    /** The name shown on the Driver Station while waiting for start. */
    protected abstract String autoName();

    protected abstract Pose startPose();

    /** Where the robot parks: inside the loading zone and NOT touching the wall (LEAVE needs that). */
    protected abstract Pose parkPose();

    /** Everything before parking. It is cut off at PARK_DEADLINE_MS so there is always time to park. */
    protected abstract Command scoringRoutine();

    // ------------------------------------------------------------------ red and blue

    /** True in the Blue... classes. Every pose in a routine is written for RED. */
    protected boolean isBlueAlliance() {
        return false;
    }

    /**
     * Makes poses from red-alliance numbers. For blue it turns each one half a turn (180 degrees)
     * around the middle of the field: (x, y, heading) becomes (144 - x, 144 - y, heading + 180).
     * That is how the BIOBUZZ field is laid out: the red garden is in the corner by the audience
     * wall, the blue garden in the opposite corner by the rear wall.
     */
    protected final PoseFactory alliancePoseFactory() {
        PoseFactory red = PoseFactory.degrees();
        return isBlueAlliance() ? red.rotateAround(FIELD_MIDDLE_INCHES, FIELD_MIDDLE_INCHES, 180) : red;
    }

    // ------------------------------------------------------------------ the whole autonomous

    public final Command autoRoutine() {
        return sequential(
                // Whatever is still running at the deadline is cut off here.
                scoringRoutine().raceWith(waitMs(PARK_DEADLINE_MS)),
                instant(this::stopMechanisms),

                // PARK: in the loading zone, not touching the wall.
                step("Park", sequential(
                        park(),
                        drive.waitUntilSettled(parkPose(), SETTLE_TIMEOUT_MS),
                        drive.stopCommand())),
                instant(() -> currentStep = "Done"));
    }

    /**
     * A straight line from wherever the robot is to the parking spot. The robot is normally at its
     * last shooting spot, but if the deadline cut the routine short it can be anywhere, so the
     * path is worked out when parking starts, not before.
     */
    private Command park() {
        return lazy(() -> {
            Pose here = drive.pose();
            Pose spot = parkPose();
            if (here.distance(spot) <= PARK_ALREADY_THERE_INCHES) return null;
            return drive.follow(Paths.line(here, spot).linear(here, spot), PARK_TIMEOUT_MS);
        });
    }

    /** Shows the step name on the Driver Station while the step runs. */
    protected final Command step(String name, Command command) {
        return sequential(instant(() -> currentStep = name), command);
    }

    protected final void stopMechanisms() {
        intake.stop();
        flywheel.stop();
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        try {
            telemetry.addLine("Starting... keep the robot STILL");
            telemetry.update();

            intake.init(hardwareMap);
            flywheel.init(hardwareMap);
            drive.init(hardwareMap, startPose());

            Pose start = startPose();
            while (opModeInInit()) {
                // Keeps reading the Pinpoint (the wheels stay off), so the numbers below are live:
                // if the robot is moved after INIT, or blue is on the wrong side, it shows here.
                drive.update();
                telemetry.addLine("READY - " + autoName());
                telemetry.addLine("Load 4 pollen. Robot on the start position.");
                telemetry.addData("Should read about", "x %.0f, y %.0f, heading %.0f",
                        start.x(), start.y(), Math.toDegrees(start.heading()));
                drive.addTelemetry(telemetry);
                telemetry.update();
                sleep(50);
            }
            if (!opModeIsActive()) return;

            autoTimer.reset();
            schedule(autoRoutine());

            while (opModeIsActive()) {
                drive.update();
                flywheel.update();
                Scheduler.execute();

                telemetry.addData("Step", currentStep);
                telemetry.addData("Time (s)", "%.1f", autoTimer.seconds());
                drive.addTelemetry(telemetry);
                flywheel.addTelemetry(telemetry);
                intake.addTelemetry(telemetry);
                telemetry.update();
            }
        } finally {
            // Runs on a normal finish, on Stop, and on an error: nothing is left spinning, and a
            // speed limit set by a path is removed for the next OpMode. Each part is in its own
            // finally so one failing cannot skip the next.
            try {
                Scheduler.reset();
                stopMechanisms();
            } finally {
                drive.stop();
            }
        }
    }
}
