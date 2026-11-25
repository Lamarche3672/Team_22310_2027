package org.firstinspires.ftc.teamcode.Shared;

import com.qualcomm.hardware.lynx.LynxController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * Helps the robot carry on after a hub loses power for a moment.
 *
 * What happens on the real robot: the battery dips, a hub restarts, and it forgets every motor
 * setting. The next motor command is then refused and the SDK throws
 * TargetPositionNotSetException (the message talks about RUN_TO_POSITION, which we never use;
 * it is just the SDK's wording for "the hub refused to enable this motor").
 *
 * Each mechanism catches that exception, calls forget() so the SDK stops trusting what it
 * thinks the hub already knows, sends its motor settings again, and retries. The count is
 * shown on the Driver Station so the drive team knows there is a power problem to chase.
 */
public final class HubReset {
    /** Several motors notice the same restart. Anything this close together is one event. */
    private static final long SAME_RESET_NANOS = 1_000_000_000L;

    private static int count;
    private static long lastNanos;

    private HubReset() { }

    /** Call at the start of every OpMode so an old count does not carry over. */
    public static void clear() {
        count = 0;
        lastNanos = 0;
    }

    public static int count() {
        return count;
    }

    /**
     * The SDK remembers the last mode/power it sent and skips re-sending "the same" value.
     * After a hub restart that memory is wrong, so wipe it for these motors' controllers.
     */
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

    /** A line for the Driver Station, or null when nothing has happened. */
    public static String warning() {
        if (count == 0) return null;
        return "A HUB LOST POWER " + count + "x (fixed itself). Check the battery, the power switch and the"
                + " power cables between the hubs.";
    }
}
