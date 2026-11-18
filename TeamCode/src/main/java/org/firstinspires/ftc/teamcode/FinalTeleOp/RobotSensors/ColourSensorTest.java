package org.firstinspires.ftc.teamcode.FinalTeleOp.RobotSensors;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Colour Sensor Test", group = "TeamCode")
public class ColourSensorTest extends OpMode {

    ColourSensor bench = new ColourSensor();
    ColourSensor.DetectedColor detectedColor;

    @Override
    public void init() {
        bench.init(hardwareMap);

    }

    @Override
    public void loop() {
        detectedColor = bench.getDetectedColor(telemetry);
        telemetry.addData("Colour Detected", detectedColor);
    }
}
