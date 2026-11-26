package org.firstinspires.ftc.teamcode.Shared;

import com.qualcomm.robotcore.exception.TargetPositionNotSetException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * The intake roller and the index belt (the "upside-down treadmill" that carries pollen from
 * the intake up to the flywheels). Teleop and autonomous both drive them through this class,
 * so the directions and speeds are written down in exactly one place.
 *
 * Callers say what they want in plain terms:
 *   intake:  +1 = pull pollen in, -1 = push it back out
 *   index:   +1 = carry pollen toward the shooter, -1 = back toward the intake
 * and this class turns that into motor powers.
 *
 * Call update() every loop: powers are ramped up over a fraction of a second instead of
 * jumping, because a motor going from 0 to full in one step pulls a current spike that the
 * battery feels. Slowing down and stopping are immediate.
 */
public final class IntakeAndIndex {
    public static final double INTAKE_POWER = 0.95;
    public static final double INDEX_FEED_POWER = 0.95;
    /**
     * Spitting pollen back out: the belt runs backwards at the same power as the intake
     * roller, so the two move pollen at the same rate and nothing bunches up between them.
     * It was 0.25 for a while, and that was not enough to turn the belt at all.
     */
    public static final double INDEX_OUTTAKE_POWER = INTAKE_POWER;

    // On this robot a NEGATIVE index motor power carries pollen toward the shooter.
    private static final double INDEX_TOWARD_SHOOTER = -1.0;

    /** Full power is reached in 1/8 of a second (slower on a tired battery). */
    private static final double POWER_RISE_PER_SECOND = 8.0;

    private final DcMotor intakeMotor;
    private final DcMotor indexMotor;
    private final Battery battery;

    private double wantedIntake;
    private double wantedIndex;
    private double intakePower;
    private double indexPower;
    private long lastUpdateNanos;

    public IntakeAndIndex(HardwareMap hardwareMap, Battery battery) {
        this.battery = battery;
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        indexMotor = hardwareMap.get(DcMotor.class, "index_motor");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        indexMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        configure();
        stop();
    }

    private void configure() {
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        indexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /** The general command. Both values are -1 to 1, see the class comment for directions. */
    public void command(double intake, double index) {
        intake = clamp(intake, -1.0, 1.0);
        index = clamp(index, -1.0, 1.0);
        wantedIntake = intake * INTAKE_POWER;
        wantedIndex = INDEX_TOWARD_SHOOTER * (index >= 0 ? index * INDEX_FEED_POWER : index * INDEX_OUTTAKE_POWER);
    }

    /** Pull pollen in. indexCreep (0 to 1) runs the belt slowly so pollen pack toward the shooter. */
    public void intake(double indexCreep) {
        command(1.0, clamp(indexCreep, 0.0, 1.0));
    }

    /** Push pollen into the flywheels. scale (0 to 1) comes from FlywheelControl.feedScale(). */
    public void feed(double scale) {
        command(1.0, clamp(scale, 0.0, 1.0));
    }

    /** Spit everything back out the way it came in. */
    public void outtake() {
        command(-1.0, -1.0);
    }

    public void stop() {
        wantedIntake = 0;
        wantedIndex = 0;
        intakePower = 0;
        indexPower = 0;
        write();
    }

    /** Call every loop. */
    public void update() {
        long now = System.nanoTime();
        double dt = lastUpdateNanos == 0 ? 0.0 : Math.min(0.05, (now - lastUpdateNanos) / 1e9);
        lastUpdateNanos = now;
        double maxRise = POWER_RISE_PER_SECOND * battery.accelLimit() * dt;
        intakePower = ramp(intakePower, wantedIntake, maxRise);
        indexPower = ramp(indexPower, wantedIndex, maxRise);
        write();
    }

    /** Speeding up is limited; slowing down, stopping and reversing-through-zero are not. */
    private static double ramp(double current, double target, double maxRise) {
        if (target * current < 0) current = 0.0;
        if (Math.abs(target) <= Math.abs(current)) return target;
        return current + clamp(target - current, -maxRise, maxRise);
    }

    private void write() {
        try {
            intakeMotor.setPower(intakePower);
            indexMotor.setPower(indexPower);
        } catch (TargetPositionNotSetException hubReset) {
            HubReset.forget(intakeMotor, indexMotor);
            try {
                configure();
                intakeMotor.setPower(intakePower);
                indexMotor.setPower(indexPower);
            } catch (TargetPositionNotSetException stillDown) {
                // The hub is still coming back. update() tries again next loop.
            }
        }
    }

    public double intakePower() {
        return intakePower;
    }

    public double indexPower() {
        return indexPower;
    }

    private static double clamp(double value, double low, double high) {
        return Math.max(low, Math.min(high, value));
    }
}
