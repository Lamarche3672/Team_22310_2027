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
import org.firstinspires.ftc.teamcode.FinalTeleOp.FinalMechanisms.HubReset;

public abstract class AutoBase extends LinearOpMode {

    protected static final double SETTLE_TIMEOUT_MS = 1500;
    protected static final double PARK_TIMEOUT_MS = 4000;
    protected static final double SHOOT_AIM_DEGREES = 10;
    protected static final double PARK_DEADLINE_MS = 24000;
    private static final double PARK_ALREADY_THERE_INCHES = 1;
    private static final double FIELD_MIDDLE_INCHES = 72;

    protected final MecanumDriveAuto drive = new MecanumDriveAuto();
    protected final IndexIntakeAuto intake = new IndexIntakeAuto();
    protected final FlywheelAuto flywheel = new FlywheelAuto();

    protected final ElapsedTime autoTimer = new ElapsedTime();
    protected String currentStep = "Waiting for start";

    protected abstract String autoName();

    protected abstract Pose startPose();

    protected abstract Pose parkPose();

    protected abstract Command scoringRoutine();

    protected boolean isBlueAlliance() {
        return false;
    }

    protected final PoseFactory alliancePoseFactory() {
        PoseFactory red = PoseFactory.degrees();
        return isBlueAlliance() ? red.rotateAround(FIELD_MIDDLE_INCHES, FIELD_MIDDLE_INCHES, 180) : red;
    }

    public final Command autoRoutine() {
        return sequential(
                scoringRoutine().raceWith(waitMs(PARK_DEADLINE_MS)),
                instant(this::stopMechanisms),

                step("Park", sequential(
                        park(),
                        drive.waitUntilSettled(parkPose(), SETTLE_TIMEOUT_MS),
                        drive.stopCommand())),
                instant(() -> currentStep = "Done"));
    }

    private Command park() {
        return lazy(() -> {
            Pose here = drive.pose();
            Pose spot = parkPose();
            if (here.distance(spot) <= PARK_ALREADY_THERE_INCHES) return null;
            return drive.follow(Paths.line(here, spot).linear(here, spot), PARK_TIMEOUT_MS);
        });
    }

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
        HubReset.clear();
        try {
            telemetry.addLine("Starting... keep the robot STILL");
            telemetry.update();

            intake.init(hardwareMap);
            flywheel.init(hardwareMap);
            drive.init(hardwareMap, startPose());

            Pose start = startPose();
            String problem = null;
            while (opModeInInit()) {
                try {
                    drive.update();
                } catch (IllegalStateException fault) {
                    try {
                        drive.init(hardwareMap, start);
                        problem = null;
                    } catch (RuntimeException stillBad) {
                        problem = stillBad.getMessage();
                    }
                }
                telemetry.addLine(problem == null ? "READY - " + autoName() : "NOT READY: " + problem);
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

                String hubWarning = HubReset.warning();
                if (hubWarning != null) telemetry.addLine(hubWarning);
                telemetry.addData("Step", currentStep);
                telemetry.addData("Time (s)", "%.1f", autoTimer.seconds());
                drive.addTelemetry(telemetry);
                flywheel.addTelemetry(telemetry);
                intake.addTelemetry(telemetry);
                telemetry.update();
            }
        } catch (RuntimeException error) {
            if (!isStopRequested()) throw error;
        } finally {
            try {
                Scheduler.reset();
                stopMechanisms();
            } finally {
                drive.stop();
            }
        }
    }
}
