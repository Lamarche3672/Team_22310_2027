package org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto;

import com.qualcomm.robotcore.exception.TargetPositionNotSetException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.FinalTeleOp.FinalMechanisms.HubReset;

public final class FlywheelControl {
    public static final double MOTOR_TICKS_PER_REV = 1120;
    public static final double MOTOR_FREE_RPM = 150;
    public static final double MAX_TICKS_PER_SECOND = MOTOR_TICKS_PER_REV * MOTOR_FREE_RPM / 60.0;
    public static final double HOLDABLE_TICKS_PER_SECOND = 0.85 * MAX_TICKS_PER_SECOND;

    public static final double DEFAULT_P = 1.0;
    public static final double DEFAULT_F = 1.0;
    public static final double DEFAULT_I = 0.5;

    public static final double NOMINAL_VOLTS = 12.0;
    public static final double SPIN_UP_VOLTS = 7.0;
    public static final double LOW_BATTERY_SPIN_UP_VOLTS = 3.5;
    public static final double FULL_SPIN_UP_ABOVE_VOLTS = 11.0;
    public static final double GENTLE_SPIN_UP_BELOW_VOLTS = 9.5;

    private static final double INTEGRATE_WITHIN = 200;
    private static final double MAX_INTEGRAL_POWER = 0.1;
    private static final double MATCH_GAIN = 2.0;
    private static final double MAX_MATCH_POWER = 0.1;
    private static final double DEAD_ENCODER_BELOW = 50;
    private static final double WRITE_THRESHOLD = 0.005;
    private static final long RESEND_NANOS = 250_000_000L;
    private static final long IDLE_READ_NANOS = 200_000_000L;
    private static final long VOLTAGE_READ_NANOS = 100_000_000L;

    private final DcMotorEx right;
    private final DcMotorEx left;
    private final VoltageSensor battery;

    private double p = DEFAULT_P;
    private double f = DEFAULT_F;
    private double i = DEFAULT_I;

    private double target;
    private double power;
    private double rightPower;
    private double leftPower;
    private double sentRight = Double.NaN;
    private double sentLeft = Double.NaN;
    private double integral;
    private double rightVelocity;
    private double leftVelocity;
    private double volts = Double.NaN;
    private long lastUpdateNanos;
    private long lastWriteNanos;
    private long lastReadNanos;
    private long lastVoltageNanos;

    public FlywheelControl(HardwareMap hardwareMap, DcMotorSimple.Direction direction) {
        right = hardwareMap.get(DcMotorEx.class, "right_flywheel");
        left = hardwareMap.get(DcMotorEx.class, "left_flywheel");
        battery = hardwareMap.voltageSensor.iterator().hasNext() ? hardwareMap.voltageSensor.iterator().next() : null;
        right.setDirection(direction);
        left.setDirection(direction);
        configure();
        stop();
    }

    private void configure() {
        for (DcMotorEx motor : new DcMotorEx[] {right, left}) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public void setGains(double p, double f, double i) {
        this.p = Math.max(0, p);
        this.f = Math.max(0, f);
        this.i = Math.max(0, i);
    }

    public void setTarget(double ticksPerSecond) {
        if (!Double.isFinite(ticksPerSecond)) ticksPerSecond = 0;
        if (Math.signum(ticksPerSecond) != Math.signum(target)) integral = 0;
        target = ticksPerSecond;
    }

    public double target() {
        return target;
    }

    public double rightVelocity() {
        return rightVelocity;
    }

    public double leftVelocity() {
        return leftVelocity;
    }

    public double power() {
        return power;
    }

    public double volts() {
        return volts;
    }

    public void stop() {
        target = 0;
        integral = 0;
        power = 0;
        rightPower = 0;
        leftPower = 0;
        write(0, 0, true);
    }

    public void update() {
        long now = System.nanoTime();
        double dt = lastUpdateNanos == 0 ? 0.0 : Math.min(0.1, (now - lastUpdateNanos) / 1e9);
        lastUpdateNanos = now;

        if (target != 0 || now - lastReadNanos >= IDLE_READ_NANOS) {
            rightVelocity = right.getVelocity();
            leftVelocity = left.getVelocity();
            lastReadNanos = now;
        }
        readBattery(now);

        power = target == 0 ? 0 : nextPower(speed(), dt);
        rightPower = matched(power, rightVelocity);
        leftPower = matched(power, leftVelocity);
        boolean changed = Math.abs(rightPower - sentRight) >= WRITE_THRESHOLD
                || Math.abs(leftPower - sentLeft) >= WRITE_THRESHOLD
                || (power == 0 && (sentRight != 0 || sentLeft != 0));
        write(rightPower, leftPower, changed || now - lastWriteNanos >= RESEND_NANOS);
    }

    private double matched(double common, double wheelVelocity) {
        if (common == 0) return 0;
        boolean bothAlive = Math.abs(rightVelocity) >= DEAD_ENCODER_BELOW && Math.abs(leftVelocity) >= DEAD_ENCODER_BELOW;
        if (!bothAlive) return common;
        double average = 0.5 * (rightVelocity + leftVelocity);
        double trim = MATCH_GAIN / 1000.0 * (average - wheelVelocity);
        trim = Math.max(-MAX_MATCH_POWER, Math.min(MAX_MATCH_POWER, trim));
        double value = common + trim;
        value = target > 0 ? Math.max(0, value) : Math.min(0, value);
        return Math.max(-1.0, Math.min(1.0, value));
    }

    public double speed() {
        double r = rightVelocity;
        double l = leftVelocity;
        if (Math.abs(r) < DEAD_ENCODER_BELOW && Math.abs(l) > 4 * DEAD_ENCODER_BELOW) return l;
        if (Math.abs(l) < DEAD_ENCODER_BELOW && Math.abs(r) > 4 * DEAD_ENCODER_BELOW) return r;
        return 0.5 * (r + l);
    }

    private double nextPower(double measured, double dt) {
        double direction = Math.signum(target);
        double compensation = Double.isFinite(volts)
                ? Math.max(0.8, Math.min(1.6, NOMINAL_VOLTS / volts)) : 1.0;
        double error = target - measured;

        if (Math.abs(error) < INTEGRATE_WITHIN) integral += error * dt;
        double integralPower = i / 1000.0 * integral;
        if (Math.abs(integralPower) > MAX_INTEGRAL_POWER) {
            integralPower = Math.signum(integralPower) * MAX_INTEGRAL_POWER;
            integral = integralPower * 1000.0 / Math.max(i, 1e-9);
        }

        double wanted = (f * target / MAX_TICKS_PER_SECOND) * compensation + p / 1000.0 * error + integralPower;
        wanted = direction > 0 ? Math.max(0, wanted) : Math.min(0, wanted);

        double supply = Double.isFinite(volts) ? volts : NOMINAL_VOLTS;
        double backEmf = measured / MAX_TICKS_PER_SECOND * NOMINAL_VOLTS / supply;
        double headroom = spinUpVolts(supply) / supply;
        wanted = Math.max(backEmf - headroom, Math.min(backEmf + headroom, wanted));
        return Math.max(-1.0, Math.min(1.0, wanted));
    }

    public static double spinUpVolts(double supply) {
        if (!Double.isFinite(supply)) return SPIN_UP_VOLTS;
        double fraction = (supply - GENTLE_SPIN_UP_BELOW_VOLTS) / (FULL_SPIN_UP_ABOVE_VOLTS - GENTLE_SPIN_UP_BELOW_VOLTS);
        fraction = Math.max(0, Math.min(1, fraction));
        return LOW_BATTERY_SPIN_UP_VOLTS + (SPIN_UP_VOLTS - LOW_BATTERY_SPIN_UP_VOLTS) * fraction;
    }

    private void readBattery(long now) {
        if (battery == null || now - lastVoltageNanos < VOLTAGE_READ_NANOS) return;
        lastVoltageNanos = now;
        double reading = battery.getVoltage();
        if (Double.isFinite(reading) && reading > 1.0) volts = reading;
    }

    private void write(double rightValue, double leftValue, boolean send) {
        if (!send) return;
        try {
            right.setPower(rightValue);
            left.setPower(leftValue);
        } catch (TargetPositionNotSetException hubReset) {
            HubReset.forget(right, left);
            try {
                configure();
                right.setPower(rightValue);
                left.setPower(leftValue);
            } catch (TargetPositionNotSetException stillDown) {
                return;
            }
        }
        sentRight = rightValue;
        sentLeft = leftValue;
        lastWriteNanos = System.nanoTime();
    }
}
