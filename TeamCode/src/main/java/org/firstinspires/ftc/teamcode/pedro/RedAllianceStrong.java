package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.pedro.PedroTuning.Constants;

@Autonomous(name = "Red Alliance Strong", group = "Autonomous")
public class RedAllianceStrong extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose point11Shoot = poseFactory.of(56, 8, 270);
    private final Pose point22CollectLeftGardenStart = poseFactory.of(56, 8, 270);
    private final Pose point22CollectLeftGarden = poseFactory.of(10.7481, 10.2282, 200);
    private final Pose turningPointStart = poseFactory.of(10.7481, 10.2282, 200);
    private final Pose turningPoint = poseFactory.of(15.6752, 42.082, 100);
    private final Pose point43GoforSecondshotStart = poseFactory.of(15.6752, 42.082, 100);
    private final Pose point43GoforSecondshot = poseFactory.of(43.0508, 118.3084, 100);
    private final Pose point54CollectflowersStart = poseFactory.of(43.0508, 118.3084, 100);
    private final Pose point54Collectflowers = poseFactory.of(42.4066, 132.2775, 45);
    private final Pose goforthirdshotStart = poseFactory.of(42.4066, 132.2775, 45);
    private final Pose goforthirdshot = poseFactory.of(56.42, 118.68, 90);
    private final Pose parkingStart = poseFactory.of(56.42, 118.68, 90);
    private final Pose parking = poseFactory.of(7.1435, 107.982, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path11Shoot()),
                follow(follower, path22CollectLeftGarden()),
                follow(follower, turningPoint()),
                follow(follower, path43GoforSecondshot()),
                follow(follower, path54Collectflowers()),
                follow(follower, goforthirdshot()),
                follow(follower, parking())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path path11Shoot() {
        return Paths.line(start, point11Shoot).constant(point11Shoot);
    }

    public Path path22CollectLeftGarden() {
        return Paths.line(point22CollectLeftGardenStart, point22CollectLeftGarden).linear(point22CollectLeftGardenStart, point22CollectLeftGarden);
    }

    public Path turningPoint() {
        return Paths.line(turningPointStart, turningPoint).linear(turningPointStart, turningPoint);
    }

    public Path path43GoforSecondshot() {
        return Paths.line(point43GoforSecondshotStart, point43GoforSecondshot).linear(point43GoforSecondshotStart, point43GoforSecondshot);
    }

    public Path path54Collectflowers() {
        return Paths.line(point54CollectflowersStart, point54Collectflowers).linear(point54CollectflowersStart, point54Collectflowers);
    }

    public Path goforthirdshot() {
        return Paths.line(goforthirdshotStart, goforthirdshot).linear(goforthirdshotStart, goforthirdshot);
    }

    public Path parking() {
        return Paths.line(parkingStart, parking).linear(parkingStart, parking);
    }
}
