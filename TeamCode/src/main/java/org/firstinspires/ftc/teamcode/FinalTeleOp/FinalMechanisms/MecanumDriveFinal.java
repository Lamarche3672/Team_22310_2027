package org.firstinspires.ftc.teamcode.FinalTeleOp.FinalMechanisms;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.exception.TargetPositionNotSetException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Shared.AutoHandoff;
import org.firstinspires.ftc.teamcode.Shared.Battery;
import org.firstinspires.ftc.teamcode.Shared.HubReset;

/**
 * The teleop mecanum drive: field-centric when the Pinpoint has a heading, robot-centric
 * when it does not.
 *
 * What happens to a stick push on its way to the wheels:
 *   1. dead zone and a curve, for fine control near the centre of the stick
 *   2. slow mode, if held
 *   3. a ramp, so power builds over a quarter of a second instead of jumping (jumping is what
 *      makes the robot jerk and the battery sag)
 *   4. rotation into the field frame using the Pinpoint heading
 *   5. the usual mecanum mix, then a cap from the battery guard if the battery is struggling
 *
 * The motors run on plain power (RUN_WITHOUT_ENCODER). RUN_USING_ENCODER without working
 * drive encoders is what once pulled the battery from 11 V down to 4 V.
 */
public class MecanumDriveFinal {
    private static final double DRIVE_DEADZONE = 0.05;
    /**
     * The stick curve is this much straight line plus the rest cubic. All cubic felt dead in
     * the middle; all linear was twitchy. 0.4 is the mix the drivers liked.
     */
    private static final double STICK_LINEAR_PART = 0.4;
    private static final double SLOW_SCALE = 0.4;
    /** Power may rise this fast: 4 per second = zero to full in a quarter of a second. */
    private static final double MAX_POWER_RISE_PER_SECOND = 4.0;
    /**
     * The auto-aim is not ramped (a ramp would make it late and wobbly), so it can ask all
     * four wheels for full power in one step. On a healthy battery that is fine. While the
     * flywheels are spinning up, or on a tired battery, that step is the surge that restarts
     * a hub, so the aim's ceiling comes down toward this with the battery guard.
     */
    private static final double AIM_POWER_WHEN_STRUGGLING = 0.6;

    // Motor writes are the slow part of a loop, so tiny changes are skipped. But every power
    // is re-sent at least a few times a second: if a hub restarts it has to hear it again.
    private static final double POWER_WRITE_THRESHOLD = 0.01;
    private static final long POWER_RESEND_NANOS = 250_000_000L;
    private static final long CURRENT_READ_NANOS = 100_000_000L;

    // If the Pinpoint misses a read or two we keep using the last heading for a moment
    // rather than flick into robot-centric and back.
    private static final double HEADING_HOLD_SECONDS = 0.25;
    /** At the start, stay still this long waiting for the first heading before giving up on it. */
    private static final double HEADING_WAIT_SECONDS = 2.0;

    private DcMotorEx frontLeftMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backRightMotor;
    private Battery battery;
    private GoBildaPinpointDriver pinpoint;

    private final double[] wheelPowers = new double[4];
    private final long[] lastWriteNanos = new long[4];
    private final double[] wheelAmps = new double[4];
    private int nextCurrentMotor;
    private long lastCurrentNanos;

    private double rampedForward;
    private double rampedStrafe;
    private double rampedRotate;
    private long lastDriveNanos;
    private boolean slow;

    private double headingRadians = Double.NaN;
    private double headingOffsetRadians;
    private double headingRate;
    private boolean headingReady;
    private boolean hadHeading;
    private long lastGoodHeadingNanos;
    private long initNanos;
    private boolean needsRezero;
    private boolean wasNearStart = true;
    private boolean forwardFromAuto;

    /** For OpModes that only have a drive (the tag align test). MAIN TELEOP shares one Battery. */
    public void init(HardwareMap hardwareMap) {
        init(hardwareMap, new Battery(hardwareMap));
    }

    public void init(HardwareMap hardwareMap, Battery battery) {
        this.battery = battery;
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "fl_motor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "fr_motor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "bl_motor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "br_motor");

        // The left side is mirrored, so "forward" is the opposite spin for those motors.
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        configureMotors();
        stop();

        headingOffsetRadians = 0;
        headingRadians = Double.NaN;
        headingRate = 0;
        headingReady = false;
        hadHeading = false;
        needsRezero = false;
        wasNearStart = true;
        slow = false;
        initNanos = System.nanoTime();

        // The Pinpoint is optional: without it we can still drive, just not field-centric.
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            // Zeroes position and recalibrates the gyro. The robot must be still for ~0.25 s.
            pinpoint.resetPosAndIMU();
        } catch (RuntimeException noPinpoint) {
            pinpoint = null;
        }

        // Straight after an auto we know which way the robot is really facing on the field,
        // so "forward" can be set for the drivers without anyone touching START.
        double fromAuto = AutoHandoff.takeForwardOffset();
        forwardFromAuto = pinpoint != null && Double.isFinite(fromAuto);
        if (forwardFromAuto) headingOffsetRadians = fromAuto;
    }

    private void configureMotors() {
        for (DcMotorEx motor : new DcMotorEx[] {frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor}) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    // ------------------------------------------------------------------ driving

    /** Robot-centric drive straight from the sticks. */
    public void drive(double forward, double strafe, double rotate) {
        driveCubic(forward, strafe, rotate, 0, true);
    }

    /** Robot-centric, but rotate is already a motor power (from a controller), not a stick. */
    public void driveWithRotatePower(double forward, double strafe, double rotatePower) {
        driveCubic(forward, strafe, rotatePower, 0, false);
    }

    /** Reads the Pinpoint, then drives field-centric. */
    public void driveFieldRelative(double forward, double strafe, double rotate) {
        updateHeading();
        driveField(forward, strafe, rotate, false);
    }

    /**
     * Field-centric drive using the heading from the last updateHeading(). rotateIsPower is
     * true when rotate comes from the auto-aim: it is then used as is, with no stick curve
     * and no ramp, because the aim controller has already decided exactly what it wants.
     */
    public void driveField(double forward, double strafe, double rotate, boolean rotateIsPower) {
        if (isWaitingForHeading()) {
            stop();
            return;
        }
        driveCubic(forward, strafe, rotate, isFieldRelative()
                ? AngleUnit.normalizeRadians(headingRadians - headingOffsetRadians) : 0, !rotateIsPower);
    }

    public void setSlow(boolean slow) {
        this.slow = slow;
    }

    public boolean isSlow() {
        return slow;
    }

    private static double stickCurve(double amount) {
        return STICK_LINEAR_PART * amount + (1.0 - STICK_LINEAR_PART) * amount * amount * amount;
    }

    private void driveCubic(double forward, double strafe, double rotate, double heading, boolean rotateIsStick) {
        // A NaN from a glitching gamepad must never reach a motor.
        if (!Double.isFinite(forward) || !Double.isFinite(strafe) || !Double.isFinite(rotate)) {
            stop();
            return;
        }

        // The curve is applied to how far the stick is pushed, not to x and y separately, so
        // the direction you push is exactly the direction you get.
        double magnitude = Math.hypot(forward, strafe);
        double scale = magnitude > DRIVE_DEADZONE ? stickCurve(Math.min(1.0, magnitude)) / magnitude : 0;
        forward *= scale;
        strafe *= scale;
        rotate = Math.max(-1.0, Math.min(1.0, rotate));
        if (rotateIsStick) {
            rotate = Math.abs(rotate) > DRIVE_DEADZONE ? Math.signum(rotate) * stickCurve(Math.abs(rotate)) : 0;
        } else {
            double ceiling = AIM_POWER_WHEN_STRUGGLING + (1.0 - AIM_POWER_WHEN_STRUGGLING) * battery.accelLimit();
            rotate = Math.max(-ceiling, Math.min(ceiling, rotate));
        }
        if (slow) {
            forward *= SLOW_SCALE;
            strafe *= SLOW_SCALE;
            if (rotateIsStick) rotate *= SLOW_SCALE;
        }

        long now = System.nanoTime();
        // Capped so that after a long pause (or a slow loop) the ramp cannot jump.
        double dt = lastDriveNanos == 0 ? 0.0 : Math.min((now - lastDriveNanos) / 1e9, 0.05);
        lastDriveNanos = now;

        // On a tired battery, build power more gently still.
        double maxRise = MAX_POWER_RISE_PER_SECOND * battery.accelLimit() * dt;
        rampTranslation(forward, strafe, maxRise);
        rampedRotate = rotateIsStick ? ramp(rampedRotate, rotate, maxRise) : rotate;

        // Turn "away from the driver" into "along the robot's nose" by rotating the stick
        // vector back through the robot's heading (heading is counter-clockwise positive).
        double robotForward = rampedForward * Math.cos(heading) - rampedStrafe * Math.sin(heading);
        double robotStrafe = rampedForward * Math.sin(heading) + rampedStrafe * Math.cos(heading);
        applyMotorPowers(robotForward, robotStrafe, rampedRotate, now);
    }

    /**
     * Ramps forward and strafe together, as one arrow. Ramping them separately made a
     * diagonal start bend toward 45 degrees for the first tenth of a second.
     * Speeding up is limited; slowing down, stopping and reversing are instant.
     */
    private void rampTranslation(double forward, double strafe, double maxRise) {
        // Reversing: drop to zero first rather than ramp down through the old direction.
        if (forward * rampedForward + strafe * rampedStrafe < 0) {
            rampedForward = 0;
            rampedStrafe = 0;
        }
        if (Math.hypot(forward, strafe) <= Math.hypot(rampedForward, rampedStrafe)) {
            rampedForward = forward;
            rampedStrafe = strafe;
            return;
        }
        double stepForward = forward - rampedForward;
        double stepStrafe = strafe - rampedStrafe;
        double step = Math.hypot(stepForward, stepStrafe);
        double part = step <= maxRise ? 1.0 : maxRise / step;
        rampedForward += stepForward * part;
        rampedStrafe += stepStrafe * part;
    }

    private static double ramp(double current, double target, double maxRise) {
        if (target * current < 0) current = 0.0;
        if (Math.abs(target) <= Math.abs(current)) return target;
        return current + Math.max(-maxRise, Math.min(maxRise, target - current));
    }

    private void applyMotorPowers(double forward, double strafe, double rotate, long now) {
        double frontLeftPower = forward + strafe + rotate;
        double backLeftPower = forward - strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backRightPower = forward + strafe - rotate;

        // If any wheel is asked for more than full power, scale them all down together so the
        // robot still moves in the direction asked for.
        double maxPower = Math.max(1.0, Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower)),
                Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))));
        double[] targets = {frontLeftPower / maxPower, backLeftPower / maxPower,
                frontRightPower / maxPower, backRightPower / maxPower};

        // Battery guard: the same trick, with the battery's limit as the ceiling. Gentle
        // driving below the limit is not touched at all.
        double strongest = 0.0;
        for (double target : targets) strongest = Math.max(strongest, Math.abs(target));
        double limit = battery.powerLimit();
        double cap = strongest > limit ? limit / strongest : 1.0;

        DcMotorEx[] motors = {frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor};
        for (int i = 0; i < 4; i++) {
            double power = targets[i] * cap;
            boolean changed = power != wheelPowers[i]
                    && (power == 0.0 || Math.abs(power - wheelPowers[i]) >= POWER_WRITE_THRESHOLD);
            if (changed || now - lastWriteNanos[i] >= POWER_RESEND_NANOS) {
                writePower(motors[i], power);
                wheelPowers[i] = power;
                lastWriteNanos[i] = now;
            }
        }
    }

    private void writePower(DcMotorEx motor, double power) {
        try {
            motor.setPower(power);
        } catch (TargetPositionNotSetException hubReset) {
            // The hub restarted and forgot its motor setup. Set it up again and retry once.
            recoverFromHubReset();
            try {
                motor.setPower(power);
            } catch (TargetPositionNotSetException stillDown) {
                // Still coming back. The periodic re-send will get it next time.
            }
        }
    }

    private void recoverFromHubReset() {
        HubReset.forget(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
        try {
            configureMotors();
        } catch (TargetPositionNotSetException stillDown) {
            // Nothing more to do this loop.
        }
        // Force every wheel to be written again on the next pass.
        java.util.Arrays.fill(lastWriteNanos, 0L);
    }

    public void stop() {
        java.util.Arrays.fill(wheelPowers, 0.0);
        rampedForward = 0.0;
        rampedStrafe = 0.0;
        rampedRotate = 0.0;
        if (frontLeftMotor != null) writePower(frontLeftMotor, 0);
        if (backLeftMotor != null) writePower(backLeftMotor, 0);
        if (frontRightMotor != null) writePower(frontRightMotor, 0);
        if (backRightMotor != null) writePower(backRightMotor, 0);
    }

    // ------------------------------------------------------------------ heading

    /** Reads the Pinpoint. Call once per loop before driveField(). */
    public void updateHeading() {
        if (pinpoint == null) {
            headingReady = false;
            return;
        }
        pinpoint.update();
        double heading = pinpoint.getHeading(AngleUnit.RADIANS);
        GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
        // A pod fault (unplugged odometry pod) does not hurt the gyro, so those statuses are
        // fine for driving. Only these four mean the heading itself cannot be trusted.
        boolean gyroUsable = status != GoBildaPinpointDriver.DeviceStatus.NOT_READY
                && status != GoBildaPinpointDriver.DeviceStatus.CALIBRATING
                && status != GoBildaPinpointDriver.DeviceStatus.FAULT_BAD_READ
                && status != GoBildaPinpointDriver.DeviceStatus.FAULT_IMU_RUNAWAY;
        headingReady = gyroUsable && Double.isFinite(heading);
        if (!headingReady) return;

        long now = System.nanoTime();
        // Spotting a Pinpoint that restarted (its cable lost power for a moment): it comes
        // back reading "at the start, facing zero" although we had driven away. Its heading
        // is then meaningless until the driver sets forward again.
        boolean nearStart = Math.abs(heading) < Math.toRadians(3)
                && Math.hypot(pinpoint.getPosX(DistanceUnit.INCH), pinpoint.getPosY(DistanceUnit.INCH)) < 1.0;
        if (hadHeading && nearStart && !wasNearStart
                && now - lastGoodHeadingNanos > HEADING_HOLD_SECONDS * 1e9) {
            needsRezero = true;
        }
        wasNearStart = nearStart;

        double rate = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
        headingRate = Double.isFinite(rate) ? rate : 0;
        headingRadians = heading;
        lastGoodHeadingNanos = now;
        hadHeading = true;
    }

    public boolean isFieldRelative() {
        return hadHeading && !needsRezero && (headingReady
                || System.nanoTime() - lastGoodHeadingNanos <= HEADING_HOLD_SECONDS * 1e9);
    }

    public boolean isWaitingForHeading() {
        return pinpoint != null && !hadHeading && System.nanoTime() - initNanos <= HEADING_WAIT_SECONDS * 1e9;
    }

    /** Makes the way the robot faces right now "forward" for field-centric driving. */
    public void resetFieldHeading() {
        updateHeading();
        if (headingReady) {
            headingOffsetRadians = headingRadians;
            needsRezero = false;
            forwardFromAuto = false;
        }
    }

    public boolean isHeadingReady() {
        return headingReady;
    }

    /** The Pinpoint's own heading in radians, before the driver's "forward" offset. */
    public double rawHeading() {
        return headingRadians;
    }

    /** How fast the robot is turning, radians per second, counter-clockwise positive. */
    public double headingRate() {
        return headingRate;
    }

    public GoBildaPinpointDriver.DeviceStatus getHeadingStatus() {
        return pinpoint == null ? GoBildaPinpointDriver.DeviceStatus.NOT_READY : pinpoint.getDeviceStatus();
    }

    public double getFieldHeadingDegrees() {
        return isFieldRelative() ? Math.toDegrees(AngleUnit.normalizeRadians(
                headingRadians - headingOffsetRadians)) : Double.NaN;
    }

    public String getDriveStatus() {
        if (isFieldRelative()) return forwardFromAuto ? "Ready (field-centric, forward set by auto)" : "Ready (field-centric)";
        if (isWaitingForHeading()) return "STOPPED: waiting for Pinpoint";
        if (pinpoint == null) return "ROBOT-CENTRIC: no Pinpoint in the robot config";
        if (needsRezero) return "ROBOT-CENTRIC: Pinpoint restarted - face forward, tap START";
        return "ROBOT-CENTRIC: Pinpoint heading unavailable";
    }

    // ------------------------------------------------------------------ telemetry

    public void addPowerTelemetry(Telemetry telemetry) {
        if (frontLeftMotor == null) return;
        // Reading a motor current is its own hub command, so only one motor each 100 ms.
        long now = System.nanoTime();
        if (now - lastCurrentNanos >= CURRENT_READ_NANOS) {
            lastCurrentNanos = now;
            DcMotorEx[] motors = {frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor};
            wheelAmps[nextCurrentMotor] = motors[nextCurrentMotor].getCurrent(CurrentUnit.AMPS);
            nextCurrentMotor = (nextCurrentMotor + 1) % motors.length;
        }
        double limit = battery.powerLimit();
        telemetry.addData("Battery (V per hub)", battery.describe());
        telemetry.addData("Drive power limit", limit < 0.999
                ? String.format("%.0f%% - battery low, power eased off", limit * 100) : "100%");
        telemetry.addData("Drive amps FL / BL / FR / BR", "%.1f / %.1f / %.1f / %.1f",
                wheelAmps[0], wheelAmps[1], wheelAmps[2], wheelAmps[3]);
    }
}
