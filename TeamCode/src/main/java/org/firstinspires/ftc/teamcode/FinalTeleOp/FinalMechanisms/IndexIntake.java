package org.firstinspires.ftc.teamcode.FinalTeleOp.FinalMechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Shared.Battery;
import org.firstinspires.ftc.teamcode.Shared.IntakeAndIndex;

/**
 * Teleop buttons for the intake and the index belt (gamepad 1).
 *
 *   RIGHT TRIGGER  intake
 *   LEFT BUMPER    index toward the shooter = shoot
 *   LEFT TRIGGER   spit everything out (beats the other two)
 *
 * The motors themselves live in Shared/IntakeAndIndex so autonomous uses the same ones.
 */
public class IndexIntake {
    private static final double TRIGGER_PRESSED = 0.1;

    private IntakeAndIndex motors;

    public void init(HardwareMap hardwareMap, Battery battery) {
        motors = new IntakeAndIndex(hardwareMap, battery);
    }

    /**
     * @param feedScale 0 to 1 from the shooter: how hard it is OK to feed right now. While the
     *                  flywheels are catching up after a pollen this drops, and the belt waits.
     */
    public void loop(Gamepad gamepad, Telemetry telemetry, double feedScale) {
        boolean outtake = gamepad.left_trigger > TRIGGER_PRESSED;
        boolean intake = gamepad.right_trigger > TRIGGER_PRESSED;
        boolean shoot = gamepad.left_bumper;

        if (outtake) {
            motors.outtake();
        } else {
            motors.command(intake ? 1.0 : 0.0, shoot ? feedScale : 0.0);
        }
        motors.update();

        telemetry.addData("Intake / index power", "%.2f / %.2f", motors.intakePower(), motors.indexPower());
        if (shoot && !outtake && feedScale < 1.0) telemetry.addLine("Index is waiting for the flywheels");
    }

    public void stop() {
        if (motors != null) motors.stop();
    }
}
