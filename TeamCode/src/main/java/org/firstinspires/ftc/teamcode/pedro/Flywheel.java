package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Flywheel - PIDF speed control for both outtake motors, battery-compensated.
 *
 *   F (feedforward) = kV x target speed + kS     -> the power it SHOULD take at 12 V
 *   P               = kP x speed error           -> push harder right after a ball drops the speed
 *   I               = kI x error built up        -> removes the small leftover error P can't fix
 *   D               = kD x change in error       -> 0 by default (encoder speed is too noisy)
 *   then everything x (12 V / battery voltage)   -> same result on a full or tired battery
 *
 * Both motors get the SAME power, so they can never fight. Speed is read from the leader's encoder.
 * Gains live in the SETTINGS block of "zack code" (AxonTeleOp).
 */
public class Flywheel {

    private final DcMotorEx leader;
    private final DcMotorEx follower;
    private final ElapsedTime timer = new ElapsedTime();

    private double integral = 0;
    private double lastError = 0;
    private double rpm = 0;
    private double targetRpm = 0;
    private double power = 0;
    private boolean on = false;

    public Flywheel(HardwareMap hardwareMap) {
        leader = hardwareMap.get(DcMotorEx.class, AxonTeleOp.LEADER_NAME);
        follower = hardwareMap.get(DcMotorEx.class, AxonTeleOp.FOLLOWER_NAME);
        leader.setDirection(AxonTeleOp.LEADER_DIR);
        follower.setDirection(AxonTeleOp.FOLLOWER_DIR);
        leader.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leader.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        follower.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leader.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        follower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /** Call once per loop. */
    public void update(boolean on, double targetRpm, Battery battery) {
        double dt = Math.min(timer.seconds(), 0.1);
        timer.reset();

        this.on = on;
        this.targetRpm = targetRpm;
        double velocity = leader.getVelocity();                       // ticks per second
        rpm = velocity / AxonTeleOp.TICKS_PER_REV * 60.0;

        if (!on) {
            integral = 0;
            lastError = 0;
            power = 0;
        } else {
            double targetTps = targetRpm / 60.0 * AxonTeleOp.TICKS_PER_REV;
            double error = targetTps - velocity;

            // I: only build up when close to speed (so spin-up doesn't overshoot), and cap it.
            double iZoneTps = AxonTeleOp.FLY_I_ZONE_RPM / 60.0 * AxonTeleOp.TICKS_PER_REV;
            if (Math.abs(error) < iZoneTps) integral += error * dt;
            if (AxonTeleOp.FLY_kI > 0) {
                double iMax = AxonTeleOp.FLY_I_MAX / AxonTeleOp.FLY_kI;
                integral = Math.max(-iMax, Math.min(iMax, integral));
            }

            double derivative = dt > 0 ? (error - lastError) / dt : 0;
            lastError = error;

            double feedforward = AxonTeleOp.FLY_kV * targetTps + AxonTeleOp.FLY_kS;
            power = feedforward
                    + AxonTeleOp.FLY_kP * error
                    + AxonTeleOp.FLY_kI * integral
                    + AxonTeleOp.FLY_kD * derivative;
            power *= battery.scale();
            power = Math.max(0.0, Math.min(1.0, power));
        }
        leader.setPower(power);
        follower.setPower(power);
    }

    public void stop() {
        leader.setPower(0);
        follower.setPower(0);
    }

    public double rpm() { return rpm; }
    public double targetRpm() { return targetRpm; }
    public double power() { return power; }
    public boolean isOn() { return on; }
    public boolean atSpeed() { return on && Math.abs(targetRpm - rpm) < AxonTeleOp.READY_RPM_TOLERANCE; }
}