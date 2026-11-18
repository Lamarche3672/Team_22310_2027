package org.firstinspires.ftc.teamcode.FinalAuto;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.FinalAuto.pedro.PedroTuning.Constants;

@Autonomous(name = "RedAllianceWeak", group = "Autonomous")
public class RedAllianceWeak extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose rotateDeadSide2 = poseFactory.of(65.222, 46.3369, 270);
    private final Pose rotateforShooting = poseFactory.of(66.0643, 95.3369, 270);
    private final Pose shootStart = poseFactory.of(66.0643, 95.3369, 270);
    private final Pose shoot = poseFactory.of(56.6, 113.4287, 90);
    private final Pose pickUpRefresh = poseFactory.of(45.4524, 133.9783, 90);
    private final Pose getFrontZoneForShooting = poseFactory.of(57.1054, 124.7381, 90);
    private final Pose parkatSpawn = poseFactory.of(7.8665, 106.3905, 90);

    // Autonomous routine
    //if (robot = pedro coordinita up top ) thnen: run intake

    public Command autoRoutine() {
        return sequential(
                follow(follower, rotateDeadSide2()),
                follow(follower, rotateforShooting()),
                follow(follower, shoot()),
                follow(follower, pickUpRefresh()),
                follow(follower, getFrontZoneForShooting()),
                follow(follower, parkatSpawn())
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

    public Path rotateDeadSide2() {
        return Paths.line(start, rotateDeadSide2).constant(rotateDeadSide2);
    }

    public Path rotateforShooting() {
        return Paths.line(rotateDeadSide2, rotateforShooting).constant(rotateforShooting);
    }

    public Path shoot() {
        return Paths.line(shootStart, shoot).linear(shootStart, shoot);
    }

    public Path pickUpRefresh() {
        return Paths.line(shoot, pickUpRefresh).constant(pickUpRefresh);
    }

    public Path getFrontZoneForShooting() {
        return Paths.line(pickUpRefresh, getFrontZoneForShooting).constant(getFrontZoneForShooting);
    }

    public Path parkatSpawn() {
        return Paths.line(getFrontZoneForShooting, parkatSpawn).constant(parkatSpawn);
    }
}

