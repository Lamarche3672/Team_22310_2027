package org.firstinspires.ftc.teamcode.FinalAuto;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.lazy;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

// The three autonomous mechanisms. Each is the auto version of the teleop class with the same name.
import org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto.FlywheelAuto;  // shooter wheels
import org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto.IndexIntakeAuto;        // intake + index
import org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto.MecanumDriveAuto;  // driving (Pedro)

/**
 * RED ALLIANCE STRONG
 *
 *   1. START            shoot the 4 preloaded pollen                 (flywheel + index)
 *   2. START -> GARDEN  drive slowly along the wall, intake running  (intake + slow index)
 *   3. TURNING POINT    swing out of the corner, intake still on     (intake + slow index)
 *   4. -> OTHER SIDE    intake off, flywheel spins up on the way     (flywheel)
 *   5. OTHER SIDE       stop, then shoot the garden pollen           (flywheel + index)
 *   6. PARK
 *
 * If something goes wrong (blocked by a wall or a robot, a flywheel that never reaches
 * speed) every step has a time limit, a blocked drive is given up after 0.6 s, and at
 * PARK_DEADLINE_MS the robot drops what it is doing and goes to park.
 */
@Autonomous(name = "Red Alliance Strong", group = "Autonomous")
public class RedAllianceStrong extends LinearOpMode {

    // ------------------------------------------------------------------ numbers to tune

    private static final double SHOOT_VELOCITY = FlywheelAuto.HIGH_VELOCITY;
    private static final double SPIN_UP_TIMEOUT_MS = 2500;
    /** How long the index pushes pollen into the flywheels for one volley of 4. */
    private static final double FEED_MS = 3000;
    /** After a pollen slows the wheels, feeding pauses this long at most before carrying on. */
    private static final double FLYWHEEL_RECOVERY_MS = 600;

    /** Start -> garden: how fast the robot drives while it intakes. Lower = more careful. */
    private static final double GARDEN_SPEED_INCHES_PER_SECOND = 20;
    /** Start -> garden: intake motor power (1.0 = same as holding the right trigger). */
    private static final double GARDEN_INTAKE_POWER = IndexIntakeAuto.INTAKE_POWER;
    /**
     * Start -> garden: the index belt creeps inwards at this fraction of its shooting speed,
     * moving each pollen further in so there is room for the next. 0 = belt off while intaking.
     * Too high and the first pollen gets pushed into the flywheels, which are not spinning.
     */
    private static final double GARDEN_INDEX_SPEED = 0.3;
    /** Keep intaking this long after the turning point so the last pollen is fully inside. */
    private static final double INTAKE_FINISH_MS = 250;

    /** Time limits per drive, so a blocked robot moves on to the next step. */
    private static final double GARDEN_TIMEOUT_MS = 4500;
    private static final double TURNING_TIMEOUT_MS = 2500;
    private static final double CROSS_TIMEOUT_MS = 5000;
    private static final double SETTLE_TIMEOUT_MS = 1500;
    private static final double PARK_TIMEOUT_MS = 4000;
    /** Autonomous is 30 s. At this point the robot stops scoring and goes to park. */
    private static final double PARK_DEADLINE_MS = 24000;
    /** The second volley only fires if the robot is within this distance of the second-shot spot. */
    private static final double SHOOT_SPOT_REACH_INCHES = 6;
    /** Within this distance of the second-shot spot the robot uses the normal parking() path. */
    private static final double PARK_PATH_REACH_INCHES = 6;
    /** Closer than this to the parking spot there is nothing left to drive. */
    private static final double PARK_ALREADY_THERE_INCHES = 1;

    // ------------------------------------------------------------------ mechanisms

    private final MecanumDriveAuto drive = new MecanumDriveAuto();
    private final IndexIntakeAuto intake = new IndexIntakeAuto();
    private final FlywheelAuto flywheel = new FlywheelAuto();

    private final ElapsedTime autoTimer = new ElapsedTime();
    private String currentStep = "Waiting for start";

    // ------------------------------------------------------------------ poses

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 270);
    private final Pose point12CollectLeftGarden = poseFactory.of(10.7481, 10.2282, 200);
    private final Pose turningPointStart = poseFactory.of(10.7481, 10.2282, 200);
    private final Pose turningPoint = poseFactory.of(13.28, 15.3665, 100);
    private final Pose point33GoforSecondshotStart = poseFactory.of(13.28, 15.3665, 100);
    private final Pose point33GoforSecondshot = poseFactory.of(42.6823, 124.02, 100);
    private final Pose parkingStart = poseFactory.of(42.6823, 124.02, 100);
    private final Pose parking = poseFactory.of(7.1435, 107.982, 0);

    // ------------------------------------------------------------------ the routine

    public Command autoRoutine() {
        Command scoring = sequential(
                // 1. BEGINNING: shoot the 4 preloads from the start position.
                step("1/6  Shoot preloads", shoot()),

                // 2. START -> GARDEN: intake on, index belt creeping inwards to make room,
                //    driving slowly so the pollen is not knocked away.
                step("2/6  Start -> garden, intaking slowly", sequential(
                        intake.intakeCommand(GARDEN_INTAKE_POWER, GARDEN_INDEX_SPEED),
                        drive.followSlowly(path12CollectLeftGarden(), GARDEN_SPEED_INCHES_PER_SECOND, GARDEN_TIMEOUT_MS))),

                // 3. TURNING POINT: swing out of the corner; intake and index finish pulling the last pollen in.
                step("3/6  Turning point", sequential(
                        drive.follow(turningPoint(), TURNING_TIMEOUT_MS),
                        waitMs(INTAKE_FINISH_MS),
                        intake.stopCommand())),

                // 4. TO THE OTHER SIDE: the flywheels spin up while driving so they are ready on arrival.
                step("4/6  Crossing, flywheel spinning up", sequential(
                        flywheel.spinUpCommand(SHOOT_VELOCITY),
                        drive.follow(path33GoforSecondshot(), CROSS_TIMEOUT_MS),
                        drive.waitUntilSettled(point33GoforSecondshot, SETTLE_TIMEOUT_MS))),

                // 5. OTHER SIDE: shoot what was collected in the garden.
                //    Only if the robot really got there; a volley from the wrong place scores nothing.
                step("5/6  Shoot garden pollen", conditional(
                        () -> drive.isNear(point33GoforSecondshot, SHOOT_SPOT_REACH_INCHES),
                        shoot(),
                        flywheel.stopCommand())));

        return sequential(
                // Whatever is still running at the deadline is cut off here.
                scoring.raceWith(waitMs(PARK_DEADLINE_MS)),
                instant(this::stopMechanisms),

                // 6. PARK
                step("6/6  Park", sequential(
                        park(),
                        drive.waitUntilSettled(parking, SETTLE_TIMEOUT_MS),
                        drive.stopCommand())),
                instant(() -> currentStep = "Done"));
    }

    /** One volley: flywheels up to speed, feed the pollen through, everything off. */
    private Command shoot() {
        return sequential(
                flywheel.spinUpCommand(SHOOT_VELOCITY),
                flywheel.waitUntilReady(SPIN_UP_TIMEOUT_MS),
                feedPollen(),
                flywheel.stopCommand());
    }

    /**
     * Feeds for FEED_MS, but only while the flywheels are at speed. Each pollen slows the
     * wheels, so the index pauses until they recover; that keeps every shot the same strength.
     * If the wheels stay off speed longer than FLYWHEEL_RECOVERY_MS it feeds anyway, so pollen
     * is never left in the robot because of a flywheel problem.
     */
    private Command feedPollen() {
        long[] startedNanos = new long[1];
        long[] lastAtSpeedNanos = new long[1];
        return new CommandBuilder()
                .setStart(() -> {
                    startedNanos[0] = System.nanoTime();
                    lastAtSpeedNanos[0] = startedNanos[0];
                })
                .setExecute(() -> {
                    long now = System.nanoTime();
                    boolean atSpeed = flywheel.isAtSpeed();
                    if (atSpeed) lastAtSpeedNanos[0] = now;
                    if (atSpeed || now - lastAtSpeedNanos[0] >= FLYWHEEL_RECOVERY_MS * 1e6) {
                        intake.feedShooter();
                    } else {
                        intake.stop();
                    }
                })
                .setDone(() -> System.nanoTime() - startedNanos[0] >= FEED_MS * 1e6)
                .setEnd(reason -> intake.stop());
    }

    /**
     * Normally the robot is at the second-shot spot and drives the parking() path.
     * If the deadline cut the routine short it can be anywhere, so it then drives a
     * straight line from where it actually is. Decided when the step starts, not before.
     */
    private Command park() {
        return lazy(() -> {
            Pose here = drive.pose();
            if (here.distance(parking) <= PARK_ALREADY_THERE_INCHES) return null;
            Path path = here.distance(parkingStart) <= PARK_PATH_REACH_INCHES
                    ? parking()
                    : Paths.line(here, parking).linear(here, parking);
            return drive.follow(path, PARK_TIMEOUT_MS);
        });
    }

    /** Shows the step name on the Driver Station while the step runs. */
    private Command step(String name, Command command) {
        return sequential(instant(() -> currentStep = name), command);
    }

    private void stopMechanisms() {
        intake.stop();
        flywheel.stop();
    }

    // ------------------------------------------------------------------ the OpMode

    @Override
    public void runOpMode() {
        Scheduler.reset();
        try {
            telemetry.addLine("Starting... keep the robot STILL");
            telemetry.update();

            intake.init(hardwareMap);
            flywheel.init(hardwareMap);
            drive.init(hardwareMap, start);

            while (opModeInInit()) {
                telemetry.addLine("READY - Red Alliance Strong");
                telemetry.addLine("Load 4 pollen. Robot on the start position.");
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
            // Runs on a normal finish, on Stop, and on an error: nothing is left spinning,
            // and the garden speed limit is removed for the next OpMode.
            Scheduler.reset();
            stopMechanisms();
            drive.stop();
        }
    }

    // ------------------------------------------------------------------ paths

    public Path path12CollectLeftGarden() {
        return Paths.line(start, point12CollectLeftGarden).linear(start, point12CollectLeftGarden);
    }

    public Path turningPoint() {
        return Paths.line(turningPointStart, turningPoint).linear(turningPointStart, turningPoint);
    }

    public Path path33GoforSecondshot() {
        return Paths.line(point33GoforSecondshotStart, point33GoforSecondshot).linear(point33GoforSecondshotStart, point33GoforSecondshot);
    }

    public Path parking() {
        return Paths.line(parkingStart, parking).linear(parkingStart, parking);
    }
}
