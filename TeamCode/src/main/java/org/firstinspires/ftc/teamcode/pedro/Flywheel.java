package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

/*
 * Flywheel - PIDF speed control for both outtake motors, battery-compensated.
 *
 *   F (feedforward) = kV x target speed + kS     -> the power it SHOULD take at 12 V
 *   P               = kP x speed error           -> push harder right after a ball drops the speed
 *   I               = kI x error built up        -> removes the leftover error P can't fix
 *   D               = kD x change in error       -> 0 by default (encoder speed is too noisy)
 *   then everything x (12 V / battery voltage)   -> same result on a full or tired battery
 *
 * After a ball knocks the speed down, P alone asks for more and more power the lower the wheel is;
 * about 200 RPM low it is already at FULL power. So recovery is as fast as the motors allow, and it
 * eases off smoothly as the wheel comes back instead of slamming on and off.
 *
 * The I part is what the wheel has LEARNED about the power it really needs to hold this speed. It
 * only learns slowly (so a ball passing through doesn't upset it) and it is kept when the flywheel
 * is switched off, so the next spin-up lands on the right speed straight away.
 *
 * Both motors get the SAME power, so they can never fight (they drive the same wheel).
 * Speed = the AVERAGE of BOTH motor encoders. If one encoder reads dead (cable out), the other is used
 * and the screen warns you.
 * Gains live in the SETTINGS block of "zack code" (AxonTeleOp).
 */
public class Flywheel {

    // Ball log tuning (see logBalls):
    private static final double BASE_BAND_RPM = 35;      // readings within this of the baseline are "just noise"
    private static final double SETTLE_SEC = 0.25;       // must hold speed this long before the log starts

    private final DcMotorEx leader;
    private final DcMotorEx follower;
    private final ElapsedTime timer = new ElapsedTime();

    private double integral = 0;
    private double lastError = 0;
    private double rpm = 0;
    private double leftRpm = 0;
    private double rightRpm = 0;
    private String encoderWarning = "";
    private double targetRpm = 0;
    private double power = 0;
    private boolean on = false;
    private boolean recovering = false;     // true while it is at FULL power catching back up
    private boolean saturatedHigh = false;  // last loop wanted more than 100% power
    private boolean saturatedLow = false;   // last loop wanted less than 0% power
    private final ElapsedTime offSpeed = new ElapsedTime();   // how long the wheel has been away from its target

    // Feedforward check: the power the wheel really needs once it is holding speed.
    private final ElapsedTime steady = new ElapsedTime();
    private double holdPower12 = 0;         // smoothed power at 12 V while holding speed
    private boolean holdPowerValid = false;

    // Shot record (kept on screen so you don't have to catch a flash):
    private boolean reachedSpeed = false;   // ignore the spin-up from 0 (true once it has HELD speed for a moment)
    private final ElapsedTime atSpeedTimer = new ElapsedTime();
    private double biggestDipRpm = 0;       // largest drop below target since R1
    private int recoveries = 0;             // how many times it had to go to FULL power to catch back up

    // Ball log: one entry per ball (cleared each time the stopper opens).
    private static class Hit {
        double beforeRpm;   // wheel speed when the ball hit
        double lowRpm;      // lowest speed it was knocked down to
        double gapSec;      // time since the previous ball (0 for the first)
    }
    private final List<Hit> hits = new ArrayList<>();
    private Hit currentHit = null;          // the dip still being measured
    private final ElapsedTime sinceHit = new ElapsedTime();
    private double baseRpm = 0;             // smoothed speed right now = the speed the next ball will hit
    private double lowRpm = 0;              // lowest speed during the current dip
    private boolean armed = false;          // true = waiting for the next ball
    private final ElapsedTime sagTimer = new ElapsedTime();   // how long the speed has sat below the baseline

    public Flywheel(HardwareMap hardwareMap) {
        leader = hardwareMap.get(DcMotorEx.class, AxonTeleOp.LEADER_NAME);
        follower = hardwareMap.get(DcMotorEx.class, AxonTeleOp.FOLLOWER_NAME);
        leader.setDirection(AxonTeleOp.LEADER_DIR);
        follower.setDirection(AxonTeleOp.FOLLOWER_DIR);
        leader.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        follower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leader.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        follower.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leader.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        follower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /** Call once per loop. */
    public void update(boolean on, double targetRpm, Battery battery) {
        double dt = Math.min(timer.seconds(), 0.1);
        timer.reset();

        boolean wasOn = this.on;
        boolean targetChanged = targetRpm != this.targetRpm;
        this.on = on;
        this.targetRpm = targetRpm;
        double velocity = readVelocity();                             // ticks per second (both encoders)
        rpm = velocity / AxonTeleOp.TICKS_PER_REV * 60.0;
        double errorRpm = targetRpm - rpm;

        if (on && !wasOn) {          // just switched ON (R1): start a fresh shot record
            biggestDipRpm = 0;
            recoveries = 0;
            resetBallLog();
        }
        if (targetChanged || (on && !wasOn)) {
            reachedSpeed = false;    // don't count the trip to a new speed as a "ball"
            holdPowerValid = false;
            steady.reset();
        }

        if (!on) {
            recovering = false;
            saturatedHigh = false;
            saturatedLow = false;
            lastError = 0;
            power = 0;
            steady.reset();
            // "integral" is deliberately KEPT: see the note at the top of this file.
        } else {
            double targetTps = targetRpm / 60.0 * AxonTeleOp.TICKS_PER_REV;
            double error = targetTps - velocity;

            boolean wasRecovering = recovering;

            // I: learns the power the wheel really needs to HOLD this speed. Rules that keep a ball
            // passing through from upsetting it:
            //   - it pauses while the wheel is knocked away from its target (a ball), and only carries
            //     on if the wheel stays off-speed for FLY_I_WAIT_SEC (then it is a real shortfall);
            //   - never while the output is already pinned at 0% or 100%;
            //   - a big error counts no more than a FLY_I_RATE_RPM one;
            //   - capped at FLY_I_MAX.
            double iZoneTps = AxonTeleOp.FLY_I_ZONE_RPM / 60.0 * AxonTeleOp.TICKS_PER_REV;
            double iRateTps = AxonTeleOp.FLY_I_RATE_RPM / 60.0 * AxonTeleOp.TICKS_PER_REV;
            boolean nearTarget = Math.abs(errorRpm) < AxonTeleOp.READY_RPM_TOLERANCE;
            if (nearTarget) offSpeed.reset();
            boolean justKnocked = !nearTarget && offSpeed.seconds() < AxonTeleOp.FLY_I_WAIT_SEC;
            boolean pinned = (saturatedHigh && error > 0) || (saturatedLow && error < 0);
            if (!pinned && !justKnocked && Math.abs(error) < iZoneTps) {
                integral += Math.max(-iRateTps, Math.min(iRateTps, error)) * dt;
            }
            if (AxonTeleOp.FLY_kI > 0) {
                double iMax = AxonTeleOp.FLY_I_MAX / AxonTeleOp.FLY_kI;
                integral = Math.max(-iMax, Math.min(iMax, integral));
            }

            double derivative = dt > 0 ? (error - lastError) / dt : 0;
            lastError = error;

            double feedforward = AxonTeleOp.FLY_kV * targetTps + AxonTeleOp.FLY_kS;
            double power12 = feedforward                      // the power this needs at exactly 12 V
                    + AxonTeleOp.FLY_kP * error
                    + AxonTeleOp.FLY_kI * integral
                    + AxonTeleOp.FLY_kD * derivative;
            power = power12 * battery.scale();
            saturatedHigh = power >= 1.0;
            saturatedLow = power <= 0.0;
            power = Math.max(0.0, Math.min(1.0, power));
            recovering = saturatedHigh;

            // Record the dips caused by balls - only once the wheel has reached speed AND stayed
            // there a moment, so settling after spin-up is never mistaken for a ball.
            if (Math.abs(errorRpm) >= AxonTeleOp.READY_RPM_TOLERANCE) atSpeedTimer.reset();
            if (!reachedSpeed && atSpeedTimer.seconds() >= SETTLE_SEC) {
                reachedSpeed = true;
                armed = true;
                baseRpm = rpm;
                lowRpm = rpm;
                sagTimer.reset();
            }
            if (reachedSpeed) {
                biggestDipRpm = Math.max(biggestDipRpm, errorRpm);
                if (recovering && !wasRecovering) recoveries++;
                logBalls(dt);
            }

            // Feedforward check: once it has held speed for a moment, remember the power it takes.
            if (recovering || Math.abs(errorRpm) > AxonTeleOp.READY_RPM_TOLERANCE) {
                steady.reset();
            } else if (steady.seconds() > 0.75) {
                holdPower12 = holdPowerValid ? holdPower12 + 0.05 * (power12 - holdPower12) : power12;
                holdPowerValid = true;
            }
        }
        leader.setPower(power);
        follower.setPower(power);
    }

    /** Average of both encoders; falls back to the working one if an encoder reads dead. */
    private double readVelocity() {
        double l = Math.abs(leader.getVelocity());     // abs: the wheel only spins one way
        double r = Math.abs(follower.getVelocity());
        double toRpm = 60.0 / AxonTeleOp.TICKS_PER_REV;
        leftRpm = l * toRpm;
        rightRpm = r * toRpm;
        double deadCheckTps = 500 / toRpm;             // only judge once the wheel is above 500 RPM
        if (Math.max(l, r) > deadCheckTps && Math.min(l, r) < 0.25 * Math.max(l, r)) {
            encoderWarning = (l < r ? AxonTeleOp.LEADER_NAME : AxonTeleOp.FOLLOWER_NAME)
                    + " encoder reads ~0 - check its encoder cable. Using the other one.";
            return Math.max(l, r);
        }
        encoderWarning = "";
        return (l + r) / 2.0;
    }

    /**
     * Detects each ball as a sudden drop; remembers the speed it hit at, how far it knocked the
     * wheel down, and how long after the previous ball it came.
     * "baseRpm" follows the wheel speed with the encoder noise smoothed out. It follows quickly
     * when the wheel is climbing (recovering) and holds still when the speed suddenly falls.
     */
    private void logBalls(double dt) {
        if (armed) {
            double diff = rpm - baseRpm;
            if (diff < -AxonTeleOp.BALL_DETECT_DROP_RPM) {          // a ball just hit the wheel
                Hit h = new Hit();
                h.beforeRpm = baseRpm;
                h.lowRpm = rpm;
                h.gapSec = hits.isEmpty() ? 0 : sinceHit.seconds();
                sinceHit.reset();
                if (hits.size() < 8) {
                    hits.add(h);
                    currentHit = h;
                } else {
                    currentHit = null;
                }
                lowRpm = rpm;
                armed = false;
            } else if (diff > BASE_BAND_RPM) {                      // climbing: keep up with it
                baseRpm += (1 - Math.exp(-dt / 0.04)) * diff;
                sagTimer.reset();
            } else if (diff >= -BASE_BAND_RPM) {                    // normal noise: smooth it out
                baseRpm += (1 - Math.exp(-dt / 0.15)) * diff;
                sagTimer.reset();
            } else if (sagTimer.seconds() > 0.4) {                  // sat lower for a while: not a ball
                baseRpm = rpm;
                sagTimer.reset();
            }                                                       // else: maybe a ball starting - hold still
        } else {
            lowRpm = Math.min(lowRpm, rpm);
            if (currentHit != null) currentHit.lowRpm = lowRpm;
            if (rpm > lowRpm + AxonTeleOp.BALL_DETECT_RISE_RPM) {   // wheel is coming back up: ready for the next ball
                armed = true;
                baseRpm = rpm;
                currentHit = null;
                sagTimer.reset();
            }
        }
    }

    /** Clear the ball log (called when the stopper opens, so each volley starts fresh). */
    public void resetBallLog() {
        hits.clear();
        currentHit = null;
        armed = true;
        baseRpm = rpm;
        lowRpm = rpm;
        sagTimer.reset();
    }

    /**
     * One entry per ball: speed when it hit (how far it knocked the wheel down, time since the last ball).
     * e.g. "1) 3150 (-240)  2) 3140 (-250, +0.28s)"
     * ONE entry with a huge drop = several balls went through too close together to tell apart.
     */
    public String ballLog() {
        if (hits.isEmpty()) return "no balls yet (clears when the stopper opens)";
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < hits.size(); i++) {
            Hit h = hits.get(i);
            sb.append(i + 1).append(") ").append(Math.round(h.beforeRpm))
                    .append(" (-").append(Math.round(h.beforeRpm - h.lowRpm));
            if (i > 0) sb.append(String.format(", +%.2fs", h.gapSec));
            sb.append(")  ");
        }
        return sb.toString().trim();
    }

    public void stop() {
        leader.setPower(0);
        follower.setPower(0);
    }

    public double rpm() { return rpm; }
    public double leftRpm() { return leftRpm; }
    public double rightRpm() { return rightRpm; }
    /** Empty when both encoders are fine. */
    public String encoderWarning() { return encoderWarning; }
    public double targetRpm() { return targetRpm; }
    public double power() { return power; }
    public boolean isOn() { return on; }
    public boolean isRecovering() { return recovering; }
    public double biggestDipRpm() { return biggestDipRpm; }
    public int recoveries() { return recoveries; }
    public int ballsLogged() { return hits.size(); }

    /** "At speed" for the SHOOT light: within READY_RPM_TOLERANCE of the target. */
    public boolean atSpeed() { return on && Math.abs(targetRpm - rpm) < AxonTeleOp.READY_RPM_TOLERANCE; }

    /** True when the wheel is close enough to the target for the paced feed to push the next ball. */
    public boolean readyToFeed() {
        return on && Math.abs(targetRpm - rpm) <= AxonTeleOp.FEED_READY_RPM;
    }

    /**
     * Feedforward check, for "Shooter Tuner". Once the wheel has held speed for about a second this
     * says what FLY_kV should be, e.g. "1.0 / 2540". Empty until then.
     */
    public String suggestedKv() {
        if (!on || !holdPowerValid || targetRpm <= 0) return "";
        double needed = holdPower12 - AxonTeleOp.FLY_kS;
        if (needed <= 0.05) return "";
        double targetTps = targetRpm / 60.0 * AxonTeleOp.TICKS_PER_REV;
        return String.format("1.0 / %.0f   (holding %.0f RPM takes %.2f power at 12 V)",
                targetTps / needed, targetRpm, holdPower12);
    }
}
