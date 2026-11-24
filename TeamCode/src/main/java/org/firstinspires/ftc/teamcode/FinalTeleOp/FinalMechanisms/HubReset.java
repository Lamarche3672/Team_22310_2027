package org.firstinspires.ftc.teamcode.FinalTeleOp.FinalMechanisms;

import com.qualcomm.hardware.lynx.LynxController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

public final class HubReset {
    private static final long SAME_RESET_NANOS = 1_000_000_000L;

    private static int count;
    private static long lastNanos;

    private HubReset() { }

    public static void clear() {
        count = 0;
        lastNanos = 0;
    }

    public static int count() {
        return count;
    }

    public static void forget(DcMotor... motors) {
        long now = System.nanoTime();
        if (count == 0 || now - lastNanos > SAME_RESET_NANOS) count++;
        lastNanos = now;
        for (DcMotor motor : motors) {
            if (motor == null) continue;
            DcMotorController controller = motor.getController();
            if (controller instanceof LynxController) ((LynxController) controller).forgetLastKnown();
        }
    }

    public static String warning() {
        if (count == 0) return null;
        return "A HUB LOST POWER " + count + "x (fixed itself). Check the battery, the power switch and the"
                + " power cables between the hubs.";
    }
}
