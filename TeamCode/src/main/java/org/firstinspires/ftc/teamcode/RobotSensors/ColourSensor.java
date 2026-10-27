package org.firstinspires.ftc.teamcode.RobotSensors;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColourSensor {
    float gainLight = 4;
    double rr = 0.06;
    double rg = 0.3;
    double rb = 0.14;
    double yr = 0.5;
    double yg = 0.9;
    double yb = 0.6;
    double br = 0.07;
    double bg = 0.12;
    double bb = 0.3;

    NormalizedColorSensor colorSensor;

    public enum DetectedColor {
        RED,
        BLUE,
        YELLOW,
        UNKNOWN
    }

    public void init (HardwareMap hwMap) {
        colorSensor = hwMap.get(NormalizedColorSensor.class, "colour_intake");
        colorSensor.setGain(gainLight);
    }

    public DetectedColor getDetectedColor(Telemetry telemetry) {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();//return four values

        float normRed, normGreen, normBlue;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;

        telemetry.addData("red", normRed);
        telemetry.addData("green", normGreen);
        telemetry.addData("blue", normBlue);

        //TODO add if statements for specific colours added
        /*
        red, green, blue

        THESE VALUES ARE FROM BROGAN PRATT, FIND OUR OWN VALUES LATER
        RED = >.1 <.2, <.11
        YELLOW = >.01, >.9, <.6
        BLUE = <.07, <.12, >.32
         */

        if (normRed > rr && normGreen < rg && normBlue < rb) {
            return DetectedColor.RED;
        }
        else if (normRed > yr && normGreen > yg && normBlue < yb) {
            return DetectedColor.YELLOW;
        }
        else if (normRed < br && normGreen < bg && normBlue > bb) {
            return DetectedColor.BLUE;
        }
        else {
            return DetectedColor.UNKNOWN;
        }
    }
}
