package org.firstinspires.ftc.teamcode.FinalTeleOp.RobotSensors.TrackingSystem;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.FinalTeleOp.FinalMechanisms.MecanumDriveFinal;
import org.firstinspires.ftc.teamcode.FinalTeleOp.RobotSensors.AprilTagLimelightTest;

@TeleOp(name = "April Tag Auto Align", group = "TeamCode")

public class AprilTagAutoAlignMecanum extends OpMode {
    private final AprilTagLimelightTest aprilTagLimelightTest = new AprilTagLimelightTest();
    private final MecanumDriveFinal drive = new MecanumDriveFinal();


    private static final int[] TARGET_TAG_IDS = {34, 35, 36, 37};

    //-------------------- PD controller -------------------------
    double kP = 0.002;
    double error = 0;
    double lastError = 0;
    double goalX = 0; // offset here
    double angleTolerance = 0.4;
    double kD = 0.0001;
    double curTime = 0;
    double lastTime = 0;
    double minTurnPower = 0.08;
    private double lastFrameStamp = -1;
    private double lastRotate = 0;
    private boolean hasLastError = false;

    // -------------- driving setup -------------------
    double forward, strafe, rotate;

    // ----------controller based PD tuning ------------------

    double[] stepSizes = {0.01, 0.001, 0.0001};
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
        forward = -gamepad2.left_stick_y;
        strafe = gamepad2.left_stick_x;
        rotate = gamepad2.right_stick_x;
        // ------------get april tag info-------------------
        aprilTagLimelightTest.update();
        LLResultTypes.FiducialResult id35 = null;
        for (int id : TARGET_TAG_IDS) {
            id35 = aprilTagLimelightTest.getTagBySpecificId(id);
            if (id35 != null) break;
        }

        //--------------------auto align rotation logic -----------

        boolean autoAligning = gamepad2.y && id35 != null;
        if (id35 != null) {
            error = id35.getTargetXDegrees() - goalX;
        }
        if(gamepad2.y) {
            if (id35 != null) {
                double frameStamp = aprilTagLimelightTest.callId.getTimestamp();
                if (frameStamp == lastFrameStamp) {
                    rotate = lastRotate;
                } else if (Math.abs(error) < angleTolerance) {
                    rotate = 0;
                    lastError = error;
                    hasLastError = true;
                    lastTime = getRuntime();
                } else {
                    double pTerm = error * kP;

                    curTime = getRuntime();
                    double dT = curTime - lastTime;
                    double dTerm = hasLastError && dT > 0 ? ((error - lastError) / dT) * kD : 0;

                    rotate = Range.clip(Math.signum(error) * minTurnPower + pTerm + dTerm, -0.4, 0.4);

                    lastError = error;
                    hasLastError = true;
                    lastTime = curTime;
                }
                lastFrameStamp = frameStamp;
                lastRotate = rotate;
            } else {
                lastTime = getRuntime();
                lastError = 0;
                hasLastError = false;
            }
        } else {
            lastError = 0;
            hasLastError = false;
            lastTime = getRuntime();
        }

        //drive our motors!
        if (autoAligning) {
            drive.driveWithRotatePower(forward, strafe, rotate);
        } else {
            drive.drive(forward, strafe, rotate);
        }

        //update P and D on the fly
        //'B' button cycles through the different step sizes for tuning precision.
        if(gamepad2.bWasPressed() && !gamepad2.start) {
            stepIndex = (stepIndex + 1) % stepSizes.length; // Modulo wraps the index back to 0.
        }

        if (gamepad2.dpadLeftWasPressed()) {
            kP = Math.max(0, kP - stepSizes[stepIndex]);
        }
        if (gamepad2.dpadRightWasPressed()) {
            kP += stepSizes[stepIndex];
        }
        if (gamepad2.dpadUpWasPressed()) {
            kD += stepSizes[stepIndex];
        }
        if (gamepad2.dpadDownWasPressed()) {
            kD = Math.max(0, kD - stepSizes[stepIndex]);
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
