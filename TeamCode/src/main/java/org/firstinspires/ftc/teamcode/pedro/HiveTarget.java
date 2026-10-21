package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.ArrayList;
import java.util.List;

/*
 * HiveTarget - finds the BIOBUZZ HIVE cell AprilTag cluster with the Limelight.
 *
 * Each HIVE cell has 4 tags in a row (36h11, 3.25 in), facing DOWN:
 *   Red scoring 30-33, Red audience 34-37, Blue audience 38-41, Blue scoring 42-45
 *   Tag centers sit at -6.50, -2.75, +2.75, +6.50 in from the cluster center.
 *
 * Every loop call update(). Then read:
 *   hasTarget()   - true if a cluster was seen recently
 *   aimErrorDeg() - degrees the cluster center is RIGHT (+) or LEFT (-) of the camera
 *   distanceIn()  - inches from the camera to the cluster center
 *   frameAgeMs()  - how old the camera picture is (AimController uses it to cancel camera delay)
 *
 * Not a TeleOp - it's a helper used by "zack code" and "Shooter Tuner".
 */
public class HiveTarget {

    public enum Alliance { ANY, RED, BLUE }

    static final int APRILTAG_PIPELINE = 0;       // pipeline slot set to AprilTag in the Limelight web page
    static final double METERS_TO_INCHES = 39.3701;
    static final double[] TAG_OFFSETS_IN = {-6.50, -2.75, 2.75, 6.50};
    static final long MAX_STALENESS_MS = 150;      // ignore frames older than this
    static final double HOLD_LAST_SECONDS = 0.5;   // keep the last target this long if it flickers out

    private final Limelight3A limelight;
    private Alliance alliance = Alliance.ANY;

    private boolean seenThisFrame = false;
    private double aimErrorDeg = 0;
    private double distanceIn = 0;
    private int clusterBase = -1;
    private int tagsSeen = 0;
    private long lastSeenNanos = 0;
    private String debug = "not started";
    private double frameAgeMs = 0;

    public HiveTarget(HardwareMap hardwareMap, String name) {
        limelight = hardwareMap.get(Limelight3A.class, name);
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();
    }

    public void setAlliance(Alliance a) { alliance = a; }
    public Alliance getAlliance() { return alliance; }

    public void stop() { limelight.stop(); }

    public void update() {
        seenThisFrame = false;
        LLResult result = limelight.getLatestResult();
        if (result == null) { debug = "no data from Limelight (not connected?)"; return; }
        if (result.getPipelineIndex() != APRILTAG_PIPELINE) {
            debug = "Limelight on pipeline " + result.getPipelineIndex() + ", need " + APRILTAG_PIPELINE;
            return;
        }
        if (result.getStaleness() > MAX_STALENESS_MS) {
            debug = "old data (" + result.getStaleness() + " ms) - camera frozen?";
            return;
        }
        if (!result.isValid()) { debug = "connected, no tag in view"; return; }

        // How old this picture is: camera capture + processing + time since it arrived.
        double age = result.getCaptureLatency() + result.getTargetingLatency() + result.getStaleness();

        // Group visible tags by cluster.
        List<LLResultTypes.FiducialResult> all = result.getFiducialResults();
        StringBuilder ids = new StringBuilder();
        for (LLResultTypes.FiducialResult t : all) ids.append(t.getFiducialId()).append(' ');
        debug = "tags in view: " + (ids.length() == 0 ? "none" : ids.toString().trim());
        double bestDist = Double.MAX_VALUE;
        for (int base = 30; base <= 42; base += 4) {
            if (!allowed(base)) continue;

            List<LLResultTypes.FiducialResult> tags = new ArrayList<>();
            for (LLResultTypes.FiducialResult t : all) {
                int id = t.getFiducialId();
                if (id >= base && id < base + 4) tags.add(t);
            }
            if (tags.isEmpty()) continue;

            double[] estimate = estimateCenter(tags, base);   // {aimDeg, distIn}; distIn <= 0 = unknown
            double sortDist = estimate[1] > 0 ? estimate[1] : 1e6;
            if (sortDist < bestDist) {
                bestDist = sortDist;
                aimErrorDeg = estimate[0];
                distanceIn = estimate[1];
                clusterBase = base;
                tagsSeen = tags.size();
                seenThisFrame = true;
            }
        }
        if (seenThisFrame) {
            lastSeenNanos = System.nanoTime();
            frameAgeMs = age;
        }
    }

    /** Estimate the cluster CENTER (not just one tag), so the aim doesn't shift when only some tags are visible. */
    private double[] estimateCenter(List<LLResultTypes.FiducialResult> tags, int base) {
        // Camera space: x = right, y = down, z = forward (inches).
        List<double[]> pts = new ArrayList<>();
        List<Double> offs = new ArrayList<>();
        double txSum = 0;
        for (LLResultTypes.FiducialResult t : tags) {
            txSum += t.getTargetXDegrees();
            Pose3D pose = t.getTargetPoseCameraSpace();
            if (pose == null) continue;
            Position p = pose.getPosition();
            pts.add(new double[]{p.x * METERS_TO_INCHES, p.y * METERS_TO_INCHES, p.z * METERS_TO_INCHES});
            offs.add(TAG_OFFSETS_IN[t.getFiducialId() - base]);
        }
        double txAvg = txSum / tags.size();
        if (pts.isEmpty()) return new double[]{txAvg, -1};   // no 3D pose: can still aim, distance unknown

        double[] center;
        if (pts.size() >= 2) {
            // Direction along the row of tags, from the two farthest-apart tags we can see.
            int a = 0, b = 0;
            for (int i = 0; i < offs.size(); i++) {
                if (offs.get(i) < offs.get(a)) a = i;
                if (offs.get(i) > offs.get(b)) b = i;
            }
            double span = offs.get(b) - offs.get(a);
            double[] pa = pts.get(a), pb = pts.get(b);
            double[] u = {(pb[0] - pa[0]) / span, (pb[1] - pa[1]) / span, (pb[2] - pa[2]) / span};
            double oa = offs.get(a);
            center = new double[]{pa[0] - oa * u[0], pa[1] - oa * u[1], pa[2] - oa * u[2]};
        } else {
            center = pts.get(0);                   // only one tag: aim at that tag
        }

        double dist = Math.sqrt(center[0] * center[0] + center[1] * center[1] + center[2] * center[2]);
        if (dist < 1) dist = -1;                   // all-zero pose = 3D not available
        double aim = (pts.size() >= 2)
                ? Math.toDegrees(Math.atan2(center[0], center[2]))
                : txAvg;
        return new double[]{aim, dist};
    }

    private boolean allowed(int base) {
        switch (alliance) {
            case RED:  return base == 30 || base == 34;
            case BLUE: return base == 38 || base == 42;
            default:   return true;
        }
    }

    public boolean hasTarget() {
        return seenThisFrame || (System.nanoTime() - lastSeenNanos) / 1e9 < HOLD_LAST_SECONDS;
    }
    public boolean seenThisFrame() { return seenThisFrame; }
    public double aimErrorDeg() { return aimErrorDeg; }
    public double distanceIn() { return distanceIn; }
    public String debug() { return debug; }
    /** How old the last camera picture was, in milliseconds (for delay-corrected aiming). */
    public double frameAgeMs() { return frameAgeMs; }

    /** Camera health line, e.g. "pipeline 0 (pipe_fiducial), 40 fps, 45C". */
    public String status() {
        LLStatus s = limelight.getStatus();
        if (s == null) return "no status";
        return String.format("pipeline %d (%s), %d fps, %.0fC",
                s.getPipelineIndex(), s.getPipelineType(), (int) s.getFps(), s.getTemp());
    }

    public int tagsSeen() { return seenThisFrame ? tagsSeen : 0; }

    public String clusterName() {
        switch (clusterBase) {
            case 30: return "RED SCORING";
            case 34: return "RED AUDIENCE";
            case 38: return "BLUE AUDIENCE";
            case 42: return "BLUE SCORING";
            default: return "none";
        }
    }
}