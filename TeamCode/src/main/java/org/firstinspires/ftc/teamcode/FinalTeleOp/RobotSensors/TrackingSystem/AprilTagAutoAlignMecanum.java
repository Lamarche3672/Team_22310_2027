package org.firstinspires.ftc.teamcode.FinalTeleOp.RobotSensors.TrackingSystem;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.FinalTeleOp.FinalMechanisms.MecanumDriveFinal;
import org.firstinspires.ftc.teamcode.FinalTeleOp.RobotSensors.AprilTagLimelightTest;

@TeleOp(name = "April Tag Limelight Test", group = "TeamCode")

public class AprilTagAutoAlignMecanum extends OpMode {
    private final AprilTagLimelightTest aprilTagLimelightTest = new AprilTagLimelightTest();
    private final MecanumDriveFinal drive = new MecanumDriveFinal();


    // Red hive, audience side: tags 34 to 37 (see AprilTagLimelightTest).
    private static final int TARGET_TAG_ID = 35;

    //-------------------- PD controller -------------------------
    double kP = 0.002;
    double error = 0;
    double lastError = 0;
    double goalX = 0; // offset here
    double angleTolerance = 0.4;
    double kD = 0.0001;
    double curTime = 0;
    double lastTime = 0;

    // -------------- driving setup -------------------
    double forward, strafe, rotate;

    // ----------controller based PD tuning ------------------

    double[] stepSizes = {0.1, 0.001, 0.0001};
    int stepIndex = 1;


    @Override
    public void init() {
        aprilTagLimelightTest.init(hardwareMap);
        drive.init(hardwareMap);

        telemetry.addLine("Initialized");
    }

    @Override
    public void start() {
        aprilTagLimelightTest.start();
        resetRuntime();
        curTime = getRuntime();
        lastTime = curTime;
    }

    @Override
    public void stop() {
        aprilTagLimelightTest.stop();
        drive.stop();
    }

    @Override
    public void loop() {
        //----------------get mecanum drive inputs ----------------
        forward = -gamepad2.left_stick_y; // pushing the stick up is negative, same as MainTeleOp
        strafe = gamepad2.left_stick_x;
        rotate = gamepad2.right_stick_x;
        // ------------get april tag info-------------------
        aprilTagLimelightTest.update();
        LLResultTypes.FiducialResult id35 = aprilTagLimelightTest.getTagBySpecificId(TARGET_TAG_ID);

        //--------------------auto align rotation logic -----------

        boolean autoAligning = gamepad2.y && id35 != null;
        if(gamepad2.y) {
            if (id35 != null) {
                // Limelight exposes a fiducial's horizontal angle in degrees: positive = tag is to the RIGHT.
                // Positive rotate turns this drive to the RIGHT, so the error must be positive then too.
                // (It was goalX - angle, which turned away from the tag. If the Limelight is mounted
                // upside down the angle flips sign: make kP negative with the d-pad.)
                error = id35.getTargetXDegrees() - goalX;

                if (Math.abs(error) < angleTolerance) {
                    rotate = 0;
                } else {
                    double pTerm = error * kP;

                    curTime = getRuntime();
                    double dT = curTime - lastTime;
                    // Two loops can land on the same timestamp; dividing by 0 would stop the drive.
                    double dTerm = dT > 0 ? ((error - lastError) / dT) * kD : 0;

                    rotate = Range.clip(pTerm + dTerm, -0.4, 0.4);

                    lastError = error;
                    lastTime = curTime;
                }
            } else {
                lastTime = getRuntime();
                lastError = 0;
            }
        } else {
            lastError = 0;
            lastTime = getRuntime();
        }

        //drive our motors!
        if (autoAligning) {
            // The controller's output is already a motor power: it must not go through the stick curve.
            drive.driveWithRotatePower(forward, strafe, rotate);
        } else {
            drive.drive(forward, strafe, rotate);
        }

        //update P and D on the fly
        //'B' button cycles through the different step sizes for tuning precision.
        if(gamepad2.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length; // Modulo wraps the index back to 0.
        }

        if (gamepad2.dpadLeftWasPressed()) {
            kP -= stepSizes[stepIndex];
        }
        if (gamepad2.dpadRightWasPressed()) {
            kP += stepSizes[stepIndex];
        }
        if (gamepad2.dpadUpWasPressed()) {
            kD += stepSizes[stepIndex];
        }
        if (gamepad2.dpadDownWasPressed()) {
            kD -= stepSizes[stepIndex];
        }

        //-----------telemetry----------
        if (id35 != null) {
            if (gamepad2.y) {
                telemetry.addLine("AUTO ALIGN");
            }
            telemetry.addData("Error", error);
        } else {
            telemetry.addLine("MANUAL Rotate Mode");
        }
        telemetry.addLine("---------------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad L/R)", kP);
        telemetry.addData("Tuning D", "%.4f (D-Pad U/D)", kD);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
    }
}
