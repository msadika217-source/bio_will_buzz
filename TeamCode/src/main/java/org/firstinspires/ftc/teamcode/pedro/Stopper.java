package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Stopper - the ball stopper (goBILDA 2000 Series torque servo, standard/position mode)
 *           + the RGB indicator light.
 *
 *   CLOSED = balls held  = light RED    (starts here)
 *   OPEN   = balls go    = light GREEN
 *
 * The servo goes to STOPPER_CLOSED_POS or STOPPER_OPEN_POS and holds there by itself.
 * Find those two numbers with "Axon Position Finder" (X = stopper). Set them just SHORT of the
 * printed hard stops so the servo isn't pushing against plastic all match.
 *
 * BOTH DEVICES ARE OPTIONAL: if "stopper" or "rgbLight" is not in the robot configuration, the
 * TeleOp still runs and just shows "not configured".
 *
 * Settings live in zack code (AxonTeleOp) SETTINGS. Used by "zack code" and "Shooter Tuner".
 */
public class Stopper {

    private final ServoImplEx servo;     // null = not in the configuration
    private final ServoImplEx light;     // null = not in the configuration
    private final ElapsedTime timer = new ElapsedTime();
    private boolean open = false;
    private boolean lastButton = false;
    private int presses = 0;

    public Stopper(HardwareMap hardwareMap) {
        servo = hardwareMap.tryGet(ServoImplEx.class, AxonTeleOp.STOPPER_NAME);
        if (servo != null) {
            servo.setPwmRange(new PwmControl.PwmRange(500, 2500));   // goBILDA 2000 full 300 deg range
        }

        light = hardwareMap.tryGet(ServoImplEx.class, AxonTeleOp.RGB_NAME);
        if (light != null) {
            light.setPwmRange(new PwmControl.PwmRange(500, 2500));
            light.setPosition(AxonTeleOp.RGB_RED);   // red while waiting = balls held
        }
    }

    /** Call in start(): go to CLOSED (red). */
    public void startClosed() {
        open = false;
        timer.reset();
        if (servo != null) servo.setPosition(AxonTeleOp.STOPPER_CLOSED_POS);
    }

    /** Call every loop with the toggle button. Each NEW press flips open <-> closed. */
    public void update(boolean toggleButton) {
        // Ignore extra "presses" for a moment after a flip: some controllers send one tap as
        // press-release-press (contact bounce), which would flip it straight back.
        if (toggleButton && !lastButton && timer.seconds() > AxonTeleOp.STOPPER_TOGGLE_LOCKOUT) {
            open = !open;
            presses++;
            timer.reset();
        }
        lastButton = toggleButton;

        if (servo != null) {
            servo.setPosition(open ? AxonTeleOp.STOPPER_OPEN_POS : AxonTeleOp.STOPPER_CLOSED_POS);
        }
        if (light != null) {
            light.setPosition(open ? AxonTeleOp.RGB_GREEN : AxonTeleOp.RGB_RED);
        }
    }

    public void stop() { }   // a position servo just holds where it is

    public boolean isOpen() { return open; }

    public String status() {
        String s = servo == null ? "stopper NOT CONFIGURED" : (open ? "OPEN (green)" : "CLOSED (red)");
        s += "  [presses: " + presses + "]";
        if (light == null) s += " | light NOT CONFIGURED";
        return s;
    }
}
