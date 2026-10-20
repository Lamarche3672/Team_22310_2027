package org.firstinspires.ftc.teamcode.RobotSensors;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ColourSensorTest extends OpMode {

    ColourSensor bench = new ColourSensor();

    @Override
    public void init() {
        bench.init(hardwareMap);

    }

    @Override
    public void loop() {
        bench.getDetectedColor(telemetry);
    }
}
