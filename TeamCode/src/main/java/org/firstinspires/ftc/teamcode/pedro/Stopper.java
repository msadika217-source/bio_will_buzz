package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Stopper - the ball stopper (goBILDA 2000 Series torque servo, standard/position mode)
 *           + the TWO RGB indicator lights ("rgbLight" and "rgblight2" - both always show the same colour).
 *
 *   CLOSED = balls held  = light RED    (starts here)
 *   OPEN   = balls go    = light GREEN
 *
 * The servo goes to STOPPER_CLOSED_POS or STOPPER_OPEN_POS and holds there by itself.
 * Find those two numbers with "Axon Position Finder" (X = stopper). Set them just SHORT of the
 * printed hard stops so the servo isn't pushing against plastic all match.
 *
 * ALL DEVICES ARE OPTIONAL: if "stopper", "rgbLight" or "rgblight2" is not in the robot
 * configuration, the TeleOp still runs and just shows "not configured".
 *
 * Settings live in zack code (AxonTeleOp) SETTINGS. Used by "zack code" and "Shooter Tuner".
 */
public class Stopper {

    private final ServoImplEx servo;     // null = not in the configuration
    private final ServoImplEx light;     // null = not in the configuration
    private final ServoImplEx light2;    // second light, same colours. null = not in the configuration
    private final ElapsedTime timer = new ElapsedTime();
    private boolean open = false;
    private boolean lastButton = false;
    private int presses = 0;

    public Stopper(HardwareMap hardwareMap) {
        servo = hardwareMap.tryGet(ServoImplEx.class, AxonTeleOp.STOPPER_NAME);
        if (servo != null) {
            servo.setPwmRange(new PwmControl.PwmRange(500, 2500));   // goBILDA 2000 full 300 deg range
        }

        light = setUpLight(hardwareMap, AxonTeleOp.RGB_NAME);
        light2 = setUpLight(hardwareMap, AxonTeleOp.RGB2_NAME);
    }

    private static ServoImplEx setUpLight(HardwareMap hardwareMap, String name) {
        ServoImplEx l = hardwareMap.tryGet(ServoImplEx.class, name);
        if (l != null) {
            l.setPwmRange(new PwmControl.PwmRange(500, 2500));
            l.setPosition(AxonTeleOp.RGB_RED);   // red while waiting = balls held
        }
        return l;
    }

    /** Call in start(): go to CLOSED (red). */
    public void startClosed() {
        open = false;
        timer.reset();
        if (servo != null) servo.setPosition(AxonTeleOp.STOPPER_CLOSED_POS);
        if (light != null) light.setPosition(AxonTeleOp.RGB_RED);     // lights match the stopper straight away
        if (light2 != null) light2.setPosition(AxonTeleOp.RGB_RED);   // (matters in auto, which doesn't call update() every loop)
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
        double colour = open ? AxonTeleOp.RGB_GREEN : AxonTeleOp.RGB_RED;
        if (light != null) light.setPosition(colour);
        if (light2 != null) light2.setPosition(colour);
    }

    /**
     * For AUTO: open (true) or close (false) the stopper straight away - no button, no lockout.
     * The lights follow it. ("zack code" never calls this; it uses update() with D-pad UP.)
     */
    public void setOpen(boolean open) {
        if (open != this.open) presses++;
        this.open = open;
        timer.reset();
        if (servo != null) servo.setPosition(open ? AxonTeleOp.STOPPER_OPEN_POS : AxonTeleOp.STOPPER_CLOSED_POS);
        double colour = open ? AxonTeleOp.RGB_GREEN : AxonTeleOp.RGB_RED;
        if (light != null) light.setPosition(colour);
        if (light2 != null) light2.setPosition(colour);
    }

    public void stop() { }   // a position servo just holds where it is

    public boolean isOpen() { return open; }

    public String status() {
        String s = servo == null ? "stopper NOT CONFIGURED" : (open ? "OPEN (green)" : "CLOSED (red)");
        s += "  [presses: " + presses + "]";
        if (light == null) s += " | light NOT CONFIGURED";
        if (light2 == null) s += " | light 2 NOT CONFIGURED";
        return s;
    }
}