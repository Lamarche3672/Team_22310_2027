package org.firstinspires.ftc.teamcode.FinalTeleOp.RobotSensors.TrackingSystem;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Turret April Tag Align", group = "TeamCode")
public class TurretAutoAlignOpMode extends OpMode {
    private static final int LIMELIGHT_PIPELINE = 8;
    /** The Limelight keeps handing back its last result forever, so anything older than this is ignored. */
    private static final long MAX_RESULT_AGE_MS = 250;

    private Limelight3A limelight;
    private final TurretTracking turret = new TurretTracking();

    // kP starts at 0.0001, far too small to move the turret. Tune it here, on gamepad 1:
    // d-pad left / right = kP, d-pad up / down = kD, B = step size.
    private final double[] stepSizes = {0.01, 0.001, 0.0001};
    private int stepIndex = 1;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
        turret.init(hardwareMap);
    }

    @Override
    public void start() {
        turret.resetTimer();
        limelight.start();
    }

    @Override
    public void loop() {
        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
        if (gamepad1.dpadRightWasPressed()) turret.setkP(turret.getkP() + stepSizes[stepIndex]);
        if (gamepad1.dpadLeftWasPressed()) turret.setkP(Math.max(0, turret.getkP() - stepSizes[stepIndex]));
        if (gamepad1.dpadUpWasPressed()) turret.setkD(turret.getkD() + stepSizes[stepIndex]);
        if (gamepad1.dpadDownWasPressed()) turret.setkD(Math.max(0, turret.getkD() - stepSizes[stepIndex]));

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid() && result.getStaleness() <= MAX_RESULT_AGE_MS) {
            double bearing = result.getTx();
            telemetry.addData("Limelight tx (deg)", bearing);
            turret.update(bearing);
        } else {
            telemetry.addData("Limelight", "No valid target");
            turret.update(Double.NaN);
        }

        telemetry.addData("Turret power", "%.3f", turret.getPower());
        telemetry.addLine("---------------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad L/R)", turret.getkP());
        telemetry.addData("Tuning D", "%.4f (D-Pad U/D)", turret.getkD());
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
    }

    @Override
    public void stop() {
        turret.stop();
        if (limelight != null) limelight.stop();
    }
}
