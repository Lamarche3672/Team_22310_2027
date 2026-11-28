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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto.FlywheelAuto;
import org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto.IndexIntakeAuto;
import org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto.MecanumDriveAuto;
import org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto.Volley;
import org.firstinspires.ftc.teamcode.Shared.AutoHandoff;
import org.firstinspires.ftc.teamcode.Shared.Battery;
import org.firstinspires.ftc.teamcode.Shared.HiveAim;
import org.firstinspires.ftc.teamcode.Shared.HubReset;


/**
 * Everything the four scoring autos have in common. A real auto (RedAllianceStrong, ...)
 * only has to say where it starts, where it parks, and what it does in between.
 *
 * What this class guarantees, whatever the routine does:
 *   - the robot is parked before the buzzer: at PARK_DEADLINE_MS the scoring routine is cut
 *     off wherever it is and the robot drives to the parking spot
 *   - intake and flywheels are off at the end, on STOP, and if anything crashes
 *   - a missing intake, flywheel or camera does not stop the auto. It still drives and parks.
 *
 * Blue autos are the red ones turned half a turn about the middle of the field, so each
 * route is written once, in red coordinates.
 */
public abstract class AutoBase extends LinearOpMode {

    /** How long to wait for the robot to come to rest on a spot before moving on anyway. */
    protected static final double SETTLE_TIMEOUT_MS = 1500;
    protected static final double PARK_TIMEOUT_MS = 4000;
    /** Autonomous is 30 s. Nothing new is driven after this point. */
    private static final double STOP_DRIVING_AT_MS = 29000;
    /** Only shoot if the robot faces within this many degrees of where the route says. */
    protected static final double SHOOT_AIM_DEGREES = 10;
    // At 25 s whatever is still being scored is cut off and the robot parks. The park drive
    // then has until STOP_DRIVING_AT_MS, which is 4 s: the longest park we have takes about 3.
    protected static final double PARK_DEADLINE_MS = 25000;
    private static final double PARK_ALREADY_THERE_INCHES = 1;
    private static final double FIELD_MIDDLE_INCHES = 72;

    protected final MecanumDriveAuto drive = new MecanumDriveAuto();
    protected final IndexIntakeAuto intake = new IndexIntakeAuto();
    protected final FlywheelAuto flywheel = new FlywheelAuto();
    protected Battery battery;
    /** The camera aim. Null when there is no Limelight; then autos shoot on the route heading alone. */
    protected HiveAim aim;
    protected final ElapsedTime autoTimer = new ElapsedTime();
    protected String currentStep = "Waiting for start";

    protected abstract String autoName();

    protected abstract Pose startPose();

    protected abstract Pose parkPose();

    /** The scoring part of the routine. Parking is added after it automatically. */
    protected abstract Command scoringRoutine();

    protected boolean isBlueAlliance() {
        return false;
    }

    /** Use this to make every pose, so the same route works for both alliances. */
    protected final PoseFactory alliancePoseFactory() {
        PoseFactory red = PoseFactory.degrees();
        return isBlueAlliance() ? red.rotateAround(FIELD_MIDDLE_INCHES, FIELD_MIDDLE_INCHES, 180) : red;
    }

    public final Command autoRoutine() {
        return sequential(
                // raceWith: whichever finishes first ends the other. So at the deadline the
                // scoring routine is interrupted and we fall through to parking.
                scoringRoutine().raceWith(waitMs(PARK_DEADLINE_MS)),
                instant(this::stopMechanisms),

                step("Park", sequential(
                        park(),
                        drive.waitUntilSettled(parkPose(), SETTLE_TIMEOUT_MS),
                        drive.stopCommand())),
                instant(() -> currentStep = "Done"));
    }

    private Command park() {
        // lazy: the path is built when parking starts, from wherever the robot is by then.
        return lazy(() -> {
            Pose here = drive.pose();
            Pose spot = parkPose();
            if (here.distance(spot) <= PARK_ALREADY_THERE_INCHES) return null;
            // The park drive may use whatever is left of the auto (up to a second before the
            // buzzer), not just PARK_TIMEOUT_MS. From the start side the park spot is a long
            // way off, and a routine that finished early should not give up on parking early.
            double untilTheEnd = STOP_DRIVING_AT_MS - autoTimer.milliseconds();
            return drive.follow(Paths.line(here, spot).linear(here, spot), Math.max(PARK_TIMEOUT_MS, untilTheEnd));
        });
    }

    /** Names a part of the routine; the name shows on the Driver Station while it runs. */
    protected final Command step(String name, Command command) {
        return sequential(instant(() -> currentStep = name), command);
    }

    /**
     * One volley: aim with the camera if it can see the hive, spin up, feed, stop.
     * Pass FlywheelAuto.HIGH_VELOCITY to shoot at the team's set speed (the one on the teleop
     * screen), or any other number for a spot that needs its own.
     */
    protected final Command volley(double velocity) {
        return Volley.shoot(flywheel, intake, drive, aim, velocity);
    }

    protected final void stopMechanisms() {
        intake.stop();
        flywheel.stop();
    }

    /**
     * Sets up everything except the drive. Returns the lines to show for anything that is
     * missing, or an empty string when it is all there.
     */
    public final String initMechanisms(HardwareMap map) {
        String missing = "";
        battery = new Battery(map);
        try {
            intake.init(map, battery);
        } catch (RuntimeException e) {
            missing += "INTAKE OFF: " + e.getMessage() + "\n";
        }
        try {
            flywheel.init(map, battery);
        } catch (RuntimeException e) {
            missing += "FLYWHEEL OFF: " + e.getMessage() + "\n";
        }
        // Intake and flywheels both gone is nearly always the Expansion Hub, not the names.
        if (!missing.isEmpty()) {
            String hub = HubReset.missingHubNote(map);
            if (hub != null) missing = hub + "\n";
        }
        try {
            HiveAim camera = new HiveAim();
            camera.init(map);
            aim = camera;
        } catch (RuntimeException e) {
            aim = null;
            missing += "CAMERA AIM OFF: " + e.getMessage() + "\n";
        }
        return missing;
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        HubReset.clear();
        // Remembered by MAIN TELEOP, which starts a few seconds after this ends.
        HiveAim.blueAlliance = isBlueAlliance();
        HiveAim.matchStart();
        try {
            telemetry.addLine("Starting... keep the robot STILL");
            telemetry.update();

            String missing = initMechanisms(hardwareMap);
            drive.init(hardwareMap, startPose());

            Pose start = startPose();
            String problem = null;
            String recovered = null;
            while (opModeInInit()) {
                // The robot can sit in INIT for minutes. If the Pinpoint glitches in that
                // time, set it up again instead of letting the whole auto die before START.
                try {
                    drive.update();
                } catch (IllegalStateException fault) {
                    try {
                        drive.init(hardwareMap, start);
                        problem = null;
                        // Keep what happened on the screen: it is ready again, but somebody
                        // should still look at that cable before the match.
                        recovered = fault.getMessage();
                    } catch (RuntimeException stillBad) {
                        problem = stillBad.getMessage();
                    }
                }
                if (aim != null) aim.update(drive.pose().heading(), true);

                telemetry.addLine(problem == null ? "READY - " + autoName() : "NOT READY: " + problem);
                if (recovered != null) telemetry.addLine("Set up again after a fault: " + recovered);
                if (!missing.isEmpty()) telemetry.addLine(missing);
                telemetry.addLine("Load 4 pollen. Robot on the start position.");
                telemetry.addData("Should read about", "x %.0f, y %.0f, heading %.0f",
                        start.x(), start.y(), Math.toDegrees(start.heading()));
                drive.addTelemetry(telemetry);
                if (aim != null) telemetry.addData("Camera", aim.view());
                telemetry.addData("Shooter", flywheel.describeSettings());
                telemetry.update();
                sleep(50);
            }
            if (!opModeIsActive()) return;

            // The match starts NOW (INIT may have been minutes ago), so the hive is in its
            // starting position from this moment.
            HiveAim.matchStart();
            autoTimer.reset();
            schedule(autoRoutine());

            while (opModeIsActive()) {
                // Order matters: read the sensors, let the mechanisms react, then let the
                // routine decide what happens next.
                drive.update();
                if (aim != null) aim.update(drive.pose().heading(), true);
                flywheel.update();
                intake.update();
                Scheduler.execute();

                String hubWarning = HubReset.warning();
                if (hubWarning != null) telemetry.addLine(hubWarning);
                if (!missing.isEmpty()) telemetry.addLine(missing);
                telemetry.addData("Step", currentStep);
                telemetry.addData("Time (s)", "%.1f", autoTimer.seconds());
                drive.addTelemetry(telemetry);
                if (aim != null) telemetry.addData("Camera", aim.view());
                flywheel.addTelemetry(telemetry);
                intake.addTelemetry(telemetry);
                telemetry.update();
            }
        } catch (RuntimeException error) {
            // Pressing STOP while the Pinpoint is still starting throws from inside the
            // driver. That is not an error worth a red popup, so only real ones are passed on.
            if (!isStopRequested()) throw error;
        } finally {
            try {
                Scheduler.reset();
                stopMechanisms();
                if (aim != null) aim.stop();
                // Tell teleop which way the robot ended up facing (see AutoHandoff).
                AutoHandoff.save(drive.pose().heading(), isBlueAlliance());
            } catch (RuntimeException noPose) {
                // The drive never got as far as having a pose. Teleop just starts as usual.
            } finally {
                drive.stop();
            }
        }
    }
}
