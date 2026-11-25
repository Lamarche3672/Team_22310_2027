package org.firstinspires.ftc.teamcode.FinalTeleOp.FinalMechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

public class HiveAim {
    public static final int PIPELINE = 0;
    public static final int FIRST_RED_TAG = 30;
    public static final int FIRST_BLUE_TAG = 38;
    public static final int TAGS_PER_HIVE = 8;
    public static final int TAGS_PER_CELL = 4;

    public static final double TX_SIGN = 1.0; // make this -1 if the robot turns AWAY from the hive
    public static final double AIM_OFFSET_DEGREES = 0; // + aims further right

    public static final double KP = 1.0;
    public static final double KD = 0.08;
    public static final double STALL_PUSH_PER_SECOND = 0.4;
    public static final double MAX_STALL_PUSH = 0.25;
    public static final double STALLED_TURN_RATE = 0.1;
    public static final double REST_DEGREES = 0.4;
    public static final double MIN_TURN_POWER = 0.05;
    public static final double MAX_TURN_POWER = 0.6;
    public static final double AIMED_DEGREES = 1.5;
    public static final double STILL_AIMED_DEGREES = 2.5;
    public static final double AIMED_TURN_RATE = 0.3;
    public static final double STILL_AIMED_TURN_RATE = 0.8;
    public static final double HOLD_SECONDS = 1.5;
    public static final double SEEN_SECONDS = 0.3;
    public static final double DISTANCE_SECONDS = 1.0;

    private static final double SWITCH_CELL_AREA_RATIO = 1.5;
    private static final double METERS_TO_INCHES = 39.3701;
    private static final int HISTORY = 64;

    public static boolean blueAlliance;

    private Limelight3A limelight;
    private volatile boolean pipelineSet;
    private volatile boolean stopped;
    private double lastStamp = Double.NaN;

    private final long[] historyNanos = new long[HISTORY];
    private final double[] historyHeading = new double[HISTORY];
    private int historyCount;
    private int historyNext;

    private boolean hasAim;
    private double aimHeading;
    private long lastSeenNanos;
    private double errorDegrees = Double.NaN;
    private double distanceInches = Double.NaN;
    private int cell = -1;
    private boolean aimed;
    private boolean wasAimed;
    private double stallPush;
    private long lastControlNanos;

    public void init(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        stopped = false;
        pipelineSet = false;
        Limelight3A camera = limelight;
        Thread setup = new Thread(() -> {
            for (int attempt = 0; attempt < 20 && !stopped && !pipelineSet; attempt++) {
                try {
                    pipelineSet = camera.pipelineSwitch(PIPELINE);
                    if (!pipelineSet) Thread.sleep(1500);
                } catch (InterruptedException end) {
                    return;
                } catch (RuntimeException retry) {
                }
            }
        });
        setup.setDaemon(true);
        setup.start();
        limelight.start();
    }

    public void stop() {
        stopped = true;
        if (limelight != null) limelight.stop();
    }

    public void update(double heading, boolean headingOk) {
        long now = System.nanoTime();
        if (headingOk) noteHeading(now, heading);
        if (limelight == null) return;

        LLResult result;
        try {
            result = limelight.getLatestResult();
        } catch (RuntimeException noResult) {
            return;
        }
        if (result == null || !result.isValid()) return;
        double stamp = result.getTimestamp();
        if (stamp == 0) stamp = result.getControlHubTimeStamp();
        if (stamp == lastStamp) return;
        lastStamp = stamp;

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) return;
        int firstTag = blueAlliance ? FIRST_BLUE_TAG : FIRST_RED_TAG;
        double[] area = new double[TAGS_PER_HIVE / TAGS_PER_CELL];
        for (LLResultTypes.FiducialResult tag : tags) {
            int index = tag.getFiducialId() - firstTag;
            if (index >= 0 && index < TAGS_PER_HIVE) area[index / TAGS_PER_CELL] += Math.max(tag.getTargetArea(), 1e-6);
        }
        int best = -1;
        for (int i = 0; i < area.length; i++) {
            if (area[i] > 0 && (best < 0 || area[i] > area[best])) best = i;
        }
        if (best < 0) return;
        if (cell >= 0 && cell < area.length && area[cell] > 0 && area[best] < SWITCH_CELL_AREA_RATIO * area[cell]) best = cell;
        cell = best;

        double txSum = 0;
        double distanceSum = 0;
        int count = 0;
        int distanceCount = 0;
        for (LLResultTypes.FiducialResult tag : tags) {
            int index = tag.getFiducialId() - firstTag;
            if (index < 0 || index >= TAGS_PER_HIVE || index / TAGS_PER_CELL != cell) continue;
            txSum += tag.getTargetXDegrees();
            count++;
            try {
                Position position = tag.getCameraPoseTargetSpace().getPosition().toUnit(DistanceUnit.METER);
                double meters = Math.sqrt(position.x * position.x + position.y * position.y + position.z * position.z);
                if (Double.isFinite(meters) && meters > 0.05) {
                    distanceSum += meters * METERS_TO_INCHES;
                    distanceCount++;
                }
            } catch (RuntimeException noPose) {
            }
        }
        double age = (result.getStaleness() + result.getCaptureLatency() + result.getTargetingLatency()) / 1000.0;
        if (!Double.isFinite(age)) age = 0;
        age = Math.max(0, Math.min(0.3, age));
        onFrame(txSum / count, distanceCount > 0 ? distanceSum / distanceCount : Double.NaN, age, now, headingOk);
    }

    void onFrame(double txDegrees, double distance, double ageSeconds, long now, boolean headingOk) {
        if (!Double.isFinite(txDegrees)) return;
        boolean wasRecent = lastSeenNanos != 0 && now - lastSeenNanos <= DISTANCE_SECONDS * 1e9;
        errorDegrees = TX_SIGN * txDegrees - AIM_OFFSET_DEGREES;
        lastSeenNanos = now;
        if (headingOk && historyCount > 0) {
            double then = headingAt(now - (long) (ageSeconds * 1e9));
            aimHeading = AngleUnit.normalizeRadians(then - Math.toRadians(errorDegrees));
            hasAim = true;
        } else {
            hasAim = false;
        }
        if (Double.isFinite(distance)) {
            distanceInches = wasRecent && Double.isFinite(distanceInches)
                    ? distanceInches + 0.3 * (distance - distanceInches) : distance;
        } else if (!wasRecent) {
            distanceInches = Double.NaN;
        }
    }

    public double rotatePower(double heading, double turnRate, boolean headingOk) {
        return rotatePower(heading, turnRate, headingOk, System.nanoTime());
    }

    double rotatePower(double heading, double turnRate, boolean headingOk, long now) {
        aimed = false;
        double dt = lastControlNanos == 0 ? 0 : Math.min(0.1, (now - lastControlNanos) / 1e9);
        lastControlNanos = now;
        if (lastSeenNanos == 0 || !headingOk || !hasAim) {
            wasAimed = false;
            stallPush = 0;
            return Double.NaN;
        }
        if ((now - lastSeenNanos) / 1e9 > HOLD_SECONDS) {
            hasAim = false;
            wasAimed = false;
            stallPush = 0;
            return Double.NaN;
        }

        double error = AngleUnit.normalizeRadians(aimHeading - heading);
        double errorDegreesNow = Math.toDegrees(Math.abs(error));
        double rate = Math.abs(turnRate);
        aimed = errorDegreesNow <= (wasAimed ? STILL_AIMED_DEGREES : AIMED_DEGREES)
                && rate <= (wasAimed ? STILL_AIMED_TURN_RATE : AIMED_TURN_RATE);
        wasAimed = aimed;

        if (errorDegreesNow <= REST_DEGREES && rate <= STALLED_TURN_RATE) {
            stallPush = 0;
            return 0;
        }
        if (stallPush * error < 0) stallPush = 0;
        if (rate < STALLED_TURN_RATE) {
            stallPush += Math.signum(error) * STALL_PUSH_PER_SECOND * dt;
            stallPush = Math.max(-MAX_STALL_PUSH, Math.min(MAX_STALL_PUSH, stallPush));
        }
        double push = Math.signum(error) * MIN_TURN_POWER * Math.min(1.0, errorDegreesNow / 3.0);
        double turnLeft = KP * error - KD * turnRate + push + stallPush;
        return -Math.max(-MAX_TURN_POWER, Math.min(MAX_TURN_POWER, turnLeft));
    }

    public void release() {
        aimed = false;
        wasAimed = false;
        stallPush = 0;
        lastControlNanos = 0;
    }

    public boolean isAimed() {
        return aimed;
    }

    public boolean seesHive() {
        return lastSeenNanos != 0 && System.nanoTime() - lastSeenNanos <= SEEN_SECONDS * 1e9;
    }

    public double distanceInches() {
        return distanceInches(System.nanoTime());
    }

    double distanceInches(long now) {
        boolean recent = lastSeenNanos != 0 && now - lastSeenNanos <= DISTANCE_SECONDS * 1e9;
        return recent ? distanceInches : Double.NaN;
    }

    public String status(boolean aiming) {
        if (limelight == null) return "OFF - no limelight in the robot config";
        if (!limelight.isConnected()) return "camera not answering (still starting, or unplugged)";
        if (!seesHive() && !(aiming && aimed)) {
            return aiming ? "can't see the " + allianceName() + " hive - turn toward it" : "hive not in view";
        }
        if (aiming && !hasAim) return "needs the Pinpoint heading - aim with the stick";
        if (aiming) return aimed ? "AIMED" : "aiming...";
        return String.format("sees the hive, %.0f deg off (hold LEFT TRIGGER to aim)", errorDegrees);
    }

    public static String allianceName() {
        return blueAlliance ? "BLUE" : "RED";
    }

    void noteHeading(long now, double heading) {
        historyNanos[historyNext] = now;
        historyHeading[historyNext] = heading;
        historyNext = (historyNext + 1) % HISTORY;
        if (historyCount < HISTORY) historyCount++;
    }

    private double headingAt(long nanos) {
        int newest = (historyNext - 1 + HISTORY) % HISTORY;
        int index = newest;
        for (int i = 0; i < historyCount; i++) {
            index = (newest - i + HISTORY) % HISTORY;
            if (historyNanos[index] <= nanos) break;
        }
        return historyHeading[index];
    }
}
