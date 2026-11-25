package org.firstinspires.ftc.teamcode.FinalAuto.pedro.PedroTuning;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Sits between Pedro Pathing and the goBILDA Pinpoint and decides which readings to believe.
 *
 * Pedro's own Pinpoint localizer trusts every read. On our robot the odd I2C read fails (the
 * Pinpoint then reports NOT_READY and all zeros), and one such read used to be enough to send
 * the tuners into "Pinpoint not ready" or to hand Pedro a pose of (0, 0, 0) mid-path.
 *
 * What this does instead:
 *   - a bad read is skipped: Pedro keeps getting the last good pose
 *   - a run of bad reads lasting MAX_BAD_STREAK_SECONDS is a real fault, and update() throws
 *     with a message that says what to check (the light colour, the port, the cable)
 *   - a Pinpoint that restarted (its cable lost power) is caught, because it comes back
 *     looking healthy but with its position and settings wiped
 *
 * It implements Pedro's Localizer, so Pedro uses it exactly like the real one.
 */
public final class PinpointGuard implements Localizer {
    /** Longest wait at INIT for the Pinpoint to become READY (calibration takes ~0.25 s). */
    public static final double READY_TIMEOUT_SECONDS = 5.0;
    /** A pod or gyro fault has to persist this long at INIT before we call it real. */
    public static final double FAULT_CONFIRM_SECONDS = 1.5;
    /** While driving: bad reads for this long means the Pinpoint is gone, not just hiccuping. */
    public static final double MAX_BAD_STREAK_SECONDS = 1.0;
    private static final int MIN_BAD_STREAK_READS = 3;

    private final String name;
    private final GoBildaPinpointDriver pinpoint;
    private final PinpointLocalizer localizer;
    private final boolean allowPodFaults;

    private MotionState goodState;
    private int goodTicksX;
    private int goodTicksY;
    private DeviceStatus status = DeviceStatus.NOT_READY;
    private boolean lastReadGood;
    private long reads;
    private long goodReads;
    private int badStreak;
    private long badStreakStart;
    private String warning;
    // The pod offsets as the Pinpoint itself reported them right after setup. Used to tell a
    // Pinpoint that restarted (offsets back to 0) from one that just missed a few reads.
    private double baseXmm;
    private double baseYmm;
    private boolean restarted;

    public static PinpointGuard open(HardwareMap map, PinpointConfig config, boolean allowPodFaults) {
        String name = config.name.get();
        GoBildaPinpointDriver pinpoint;
        try {
            pinpoint = map.get(GoBildaPinpointDriver.class, name);
        } catch (RuntimeException error) {
            throw new IllegalStateException("No goBILDA Pinpoint called \"" + name + "\" in the active"
                    + " Robot Configuration. On the Driver Station open Configure Robot and add a"
                    + " \"goBILDA Pinpoint Odometry Computer\" named " + name + " on the I2C port it"
                    + " is plugged into, then Save and activate that configuration.", error);
        }

        PinpointLocalizer localizer;
        try {
            localizer = new PinpointLocalizer(map, config);
        } catch (RuntimeException error) {
            if (error.getCause() instanceof InterruptedException) {
                Thread.currentThread().interrupt();
                throw new IllegalStateException("Stopped while the Pinpoint was starting.", error);
            }
            throw error;
        }

        PinpointGuard guard = new PinpointGuard(name, pinpoint, localizer, allowPodFaults);
        guard.awaitReady();
        try {
            guard.baseXmm = pinpoint.getXOffset(DistanceUnit.MM);
            guard.baseYmm = pinpoint.getYOffset(DistanceUnit.MM);
        } catch (RuntimeException ignored) {
        }
        return guard;
    }

    private PinpointGuard(String name, GoBildaPinpointDriver pinpoint, PinpointLocalizer localizer,
                          boolean allowPodFaults) {
        this.name = name;
        this.pinpoint = pinpoint;
        this.localizer = localizer;
        this.allowPodFaults = allowPodFaults;
        this.goodState = localizer.state();
    }

    @Override
    public void update() {
        read();
        // STOP was pressed: reads fail while the OpMode shuts down, and that is not a fault.
        if (Thread.currentThread().isInterrupted()) return;
        if (badStreak >= MIN_BAD_STREAK_READS
                && secondsSince(badStreakStart) >= MAX_BAD_STREAK_SECONDS)
            throw new IllegalStateException(explain());
    }

    /** Always the last GOOD pose and velocity, never a failed read. */
    @Override
    public MotionState state() {
        return goodState;
    }

    @Override
    public void setPose(Pose pose) {
        localizer.setPose(pose);
        goodState = localizer.state();
    }

    @Override
    public void reset() {
        localizer.reset();
        awaitReady();
    }

    public DeviceStatus status() {
        return status;
    }

    public boolean lastReadGood() {
        return lastReadGood;
    }

    public int ticksX() {
        return goodTicksX;
    }

    public int ticksY() {
        return goodTicksY;
    }

    public int badStreak() {
        return badStreak;
    }

    public long skippedReads() {
        return reads - goodReads;
    }

    public String warning() {
        return warning;
    }

    private void read() {
        localizer.update();
        status = pinpoint.getDeviceStatus();
        reads++;
        // The tuners allow pod faults on purpose: finding a pod that is not detected is
        // exactly what they are for. Autonomous does not.
        boolean good = status == DeviceStatus.READY || (allowPodFaults && isPodFault(status));
        // First healthy-looking read after an outage: make sure it is the same Pinpoint
        // state we set up, not a freshly rebooted one.
        if (good && badStreak > 0 && lostConfig()) good = false;
        lastReadGood = good;
        if (lastReadGood) {
            goodReads++;
            badStreak = 0;
            goodState = localizer.state();
            goodTicksX = pinpoint.getEncoderX();
            goodTicksY = pinpoint.getEncoderY();
        } else if (badStreak++ == 0) {
            badStreakStart = System.nanoTime();
        }
    }

    /** True if the pod offsets no longer match what we set: the Pinpoint has rebooted. */
    private boolean lostConfig() {
        // The tuner runs before offsets exist (both 0), so there is nothing to compare.
        if (Math.abs(baseXmm) < 1 && Math.abs(baseYmm) < 1) return false;
        try {
            restarted = Math.abs(pinpoint.getXOffset(DistanceUnit.MM) - baseXmm) > 1
                    || Math.abs(pinpoint.getYOffset(DistanceUnit.MM) - baseYmm) > 1;
        } catch (RuntimeException error) {
            restarted = true;
        }
        return restarted;
    }

    private void awaitReady() {
        long start = System.nanoTime();
        long faultStart = 0;
        int readyInARow = 0;
        badStreak = 0;
        while (!Thread.currentThread().isInterrupted()) {
            read();
            if (status == DeviceStatus.READY) {
                if (++readyInARow >= 2) {
                    warning = null;
                    return;
                }
            } else {
                readyInARow = 0;
            }

            if (isPodFault(status) || status == DeviceStatus.FAULT_IMU_RUNAWAY) {
                if (faultStart == 0) faultStart = System.nanoTime();
                if (secondsSince(faultStart) >= FAULT_CONFIRM_SECONDS) {
                    if (allowPodFaults && isPodFault(status)) {
                        warning = faultHelp(status);
                        badStreak = 0;
                        return;
                    }
                    throw new IllegalStateException(explain());
                }
            } else {
                faultStart = 0;
            }

            if (secondsSince(start) >= READY_TIMEOUT_SECONDS)
                throw new IllegalStateException(explain());
            try {
                Thread.sleep(20);
            } catch (InterruptedException stopPressed) {
                Thread.currentThread().interrupt();
            }
        }
    }

    private static boolean isPodFault(DeviceStatus status) {
        return status == DeviceStatus.FAULT_X_POD_NOT_DETECTED
                || status == DeviceStatus.FAULT_Y_POD_NOT_DETECTED
                || status == DeviceStatus.FAULT_NO_PODS_DETECTED;
    }

    private static double secondsSince(long nanos) {
        return (System.nanoTime() - nanos) / 1e9;
    }

    private String explain() {
        DeviceStatus seen = status;
        int id = 0;
        int firmware = 0;
        try {
            id = pinpoint.getDeviceID();
            firmware = pinpoint.getDeviceVersion();
        } catch (RuntimeException ignored) {
        }
        if (restarted)
            return "The Pinpoint restarted (its cable lost power for a moment) and forgot where the"
                    + " robot is. Check the Pinpoint cable at both ends.";
        String facts = " [status " + seen + ", good reads " + goodReads + " of " + reads
                + ", device id " + id + ", firmware " + firmware + "]";
        if (id == 0 && firmware == 0)
            return "The Control Hub gets no answer from the Pinpoint \"" + name + "\". In Configure"
                    + " Robot, check it is a \"goBILDA Pinpoint Odometry Computer\" on the SAME I2C"
                    + " port number its cable is plugged into, and that the Pinpoint light is on. If it worked"
                    + " a moment ago, the cable or the hub dropped out." + facts;
        return faultHelp(seen) + facts;
    }

    private static String faultHelp(DeviceStatus status) {
        switch (status) {
            case FAULT_X_POD_NOT_DETECTED:
                return "Pinpoint light BLUE: it cannot see the forward pod on its X port.";
            case FAULT_Y_POD_NOT_DETECTED:
                return "Pinpoint light ORANGE: it cannot see the sideways pod on its Y port.";
            case FAULT_NO_PODS_DETECTED:
                return "Pinpoint light PURPLE: it cannot see either pod.";
            case FAULT_IMU_RUNAWAY:
                return "The Pinpoint gyro reports a fault. Power the robot off and on, keep it"
                        + " still while it starts, and try again.";
            case CALIBRATING:
                return "The Pinpoint is still calibrating. The robot must be completely still"
                        + " while it starts. Let go of the robot and try again.";
            case FAULT_BAD_READ:
                return "The Pinpoint answers, but its data keeps arriving scrambled. Try a"
                        + " different I2C port on the Control Hub (not port 0) or another cable.";
            default:
                return "The Pinpoint answers but never became READY. Power the robot off and"
                        + " on, keep it still while it starts, and try again.";
        }
    }
}
