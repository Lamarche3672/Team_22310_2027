package org.firstinspires.ftc.teamcode.FinalAuto;

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
import org.firstinspires.ftc.teamcode.FinalTeleOp.FinalMechanisms.FlywheelTesting;
import org.firstinspires.ftc.teamcode.FinalTeleOp.IndexIntake;
import org.firstinspires.ftc.teamcode.FinalTeleOp.MecanumDriveFinal;

public class AutoControls {
    private final MecanumDriveFinal drive = new MecanumDriveFinal();
    private final IndexIntake intakeIndex = new IndexIntake();
    private final FlywheelTesting flywheelTesting = new FlywheelTesting();

}
