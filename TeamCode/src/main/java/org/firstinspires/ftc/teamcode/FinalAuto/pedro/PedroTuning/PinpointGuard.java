package org.firstinspires.ftc.teamcode.FinalAuto.pedro.PedroTuning;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus;
import com.qualcomm.robotcore.hardware.HardwareMap;

public final class PinpointGuard implements Localizer {
    public static final double READY_TIMEOUT_SECONDS = 5.0;
    public static final double FAULT_CONFIRM_SECONDS = 1.5;
    public static final double MAX_BAD_STREAK_SECONDS = 1.0;
    private static final int MIN_BAD_STREAK_READS = 10;

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
        if (Thread.currentThread().isInterrupted()) return;
        if (badStreak >= MIN_BAD_STREAK_READS
                && secondsSince(badStreakStart) >= MAX_BAD_STREAK_SECONDS)
            throw new IllegalStateException(explain());
    }

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
        lastReadGood = status == DeviceStatus.READY || (allowPodFaults && isPodFault(status));
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
        String facts = " [status " + seen + ", good reads " + goodReads + " of " + reads
                + ", device id " + id + ", firmware " + firmware + "]";
        if (id == 0 && firmware == 0)
            return "The Control Hub gets no answer from the Pinpoint \"" + name + "\". In Configure"
                    + " Robot, check it is a \"goBILDA Pinpoint Odometry Computer\" on the SAME I2C"
                    + " port number its cable is plugged into, and that the Pinpoint light is on." + facts;
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
