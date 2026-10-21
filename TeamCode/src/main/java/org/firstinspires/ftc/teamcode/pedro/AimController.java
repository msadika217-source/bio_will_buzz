package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * AimController - smooth auto-aim using the Limelight + the Control Hub's IMU (gyro).
 *
 * WHY: the camera picture is ~20-30 ms old and only updates ~30-90 times a second, so steering
 * straight off the camera chases where the tag USED to be -> wobble.
 * FIX: the camera says "the HIVE is at heading X". The gyro (fast, no delay) turns the robot to X
 * and holds it, even if the tag flickers out for a moment.
 *
 * Turn power = PDF on heading error, battery-compensated:
 *   P = AIM_KP x degrees off      D = AIM_KD x turn speed (brakes, stops overshoot)
 *   F = AIM_KS (just enough to beat friction), only when not locked
 *
 * Also gives heading() for field-centric driving.
 * If the IMU fails to start (wrong hub orientation setting), it falls back to camera-only aiming.
 */
public class AimController {

    private static final int HISTORY = 128;

    private IMU imu;
    private boolean imuOk = false;
    private String imuError = "";

    // Live-tunable gains (start from zack code SETTINGS; Aim Tuner changes them).
    public double kP = AxonTeleOp.AIM_KP;
    public double kD = AxonTeleOp.AIM_KD;
    public double kS = AxonTeleOp.AIM_KS;
    public double maxPower = AxonTeleOp.AIM_MAX_POWER;

    private final double[] histTime = new double[HISTORY];
    private final double[] histHeading = new double[HISTORY];
    private int histIndex = 0, histCount = 0;

    private double heading = 0;      // degrees, + = robot turned LEFT
    private double yawRate = 0;      // degrees per second, + = turning LEFT
    private boolean hasAim = false;
    private double aimHeading = 0;
    private double cameraError = 0;  // used only in camera-only fallback
    private boolean locked = false;

    public AimController(HardwareMap hardwareMap) {
        try {
            imu = hardwareMap.get(IMU.class, AxonTeleOp.IMU_NAME);
            imu.initialize(new IMU.Parameters(
                    new RevHubOrientationOnRobot(AxonTeleOp.HUB_LOGO, AxonTeleOp.HUB_USB)));
            imu.resetYaw();
            imuOk = true;
        } catch (Exception e) {
            imuOk = false;
            imuError = e.getMessage();
        }
    }

    public boolean imuOk() { return imuOk; }
    public String imuError() { return imuError; }

    /** Make the direction the robot faces right now = heading 0. */
    public void resetHeading() {
        if (imuOk) imu.resetYaw();
        histCount = 0;
        clearTarget();
    }

    /** Call FIRST every loop. */
    public void update() {
        if (!imuOk) return;
        heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        yawRate = imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate;
        histTime[histIndex] = System.nanoTime() / 1e9;
        histHeading[histIndex] = heading;
        histIndex = (histIndex + 1) % HISTORY;
        if (histCount < HISTORY) histCount++;
    }

    /**
     * Give it a fresh camera reading.
     * @param errorDeg  how far RIGHT (+) or LEFT (-) of center the HIVE is in the picture
     * @param frameAgeMs how old that picture is
     */
    public void cameraUpdate(double errorDeg, double frameAgeMs) {
        cameraError = errorDeg;
        if (imuOk) {
            // Where was the robot pointing when the picture was TAKEN? Aim from there.
            double headingThen = headingAt(System.nanoTime() / 1e9 - frameAgeMs / 1000.0);
            aimHeading = AngleUnit.normalizeDegrees(headingThen - errorDeg);
        }
        hasAim = true;
    }

    /** Forget the target (call when the aim button is released). */
    public void clearTarget() {
        hasAim = false;
        locked = false;
    }

    public boolean hasAim() { return hasAim; }

    /** Degrees to turn to face the HIVE. + = turn LEFT. */
    public double error() {
        if (!hasAim) return 0;
        return imuOk ? AngleUnit.normalizeDegrees(aimHeading - heading) : -cameraError;
    }

    /** Turn power for the drivetrain (+ = turn LEFT), already battery-compensated. */
    public double turnPower(Battery battery) {
        if (!hasAim) return 0;
        double err = error();
        double tol = AxonTeleOp.AIM_TOLERANCE;

        // Lock when close and nearly stopped; unlock only if it drifts well off (no chatter).
        if (locked && Math.abs(err) > 2 * tol) locked = false;
        if (!locked && Math.abs(err) < tol && Math.abs(yawRate) < 20) locked = true;

        double p = kP * err - kD * yawRate;
        if (!locked) p += Math.copySign(kS, err);
        p *= battery.scale();
        return Math.max(-maxPower, Math.min(maxPower, p));
    }

    public boolean isLocked() { return hasAim && locked; }
    public double heading() { return heading; }
    public double yawRate() { return yawRate; }
    public double aimHeading() { return aimHeading; }

    private double headingAt(double time) {
        if (histCount == 0) return heading;
        double best = heading, bestDiff = Double.MAX_VALUE;
        for (int i = 0; i < histCount; i++) {
            double d = Math.abs(histTime[i] - time);
            if (d < bestDiff) { bestDiff = d; best = histHeading[i]; }
        }
        return best;
    }
}