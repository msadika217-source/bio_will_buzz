package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/*
 * Axon Position Finder
 * Use this FIRST to find the numbers for the SETTINGS block in AxonTeleOp.
 *
 *   A / B           : choose which servo you're moving (A = stopper, B = flower arm)
 *   D-pad Up / Down : move 0.01
 *   Bumpers         : move 0.05
 *
 * Servos stay limp until you press a D-pad or bumper, so nothing jumps on Start.
 * Write down the "Position" number for each spot you need.
 */
@TeleOp(name = "Axon Position Finder", group = "Test")
public class AxonPositionFinder extends OpMode {

    // Same names as AxonTeleOp's SETTINGS block.
    static final String STOPPER_NAME    = "stopper";
    static final String FLOWER_ARM_NAME = "flowerArm";

    private ServoImplEx[] servos;
    private final String[] labels = {"STOPPER", "FLOWER ARM"};
    private final double[] positions = {0.5, 0.5};
    private final boolean[] started = {false, false};
    private int selected = 0;

    private boolean lastUp, lastDown, lastRb, lastLb;

    @Override
    public void init() {
        servos = new ServoImplEx[]{
                hardwareMap.get(ServoImplEx.class, STOPPER_NAME),
                hardwareMap.get(ServoImplEx.class, FLOWER_ARM_NAME)
        };
        for (ServoImplEx s : servos) {
            s.setPwmRange(new PwmControl.PwmRange(500, 2500));
            s.setPwmDisable();
        }
    }

    @Override
    public void loop() {
        if (gamepad1.a) selected = 0;
        if (gamepad1.b) selected = 1;

        double step = 0;
        if (gamepad1.dpad_up && !lastUp)          step = 0.01;
        if (gamepad1.dpad_down && !lastDown)      step = -0.01;
        if (gamepad1.right_bumper && !lastRb)     step = 0.05;
        if (gamepad1.left_bumper && !lastLb)      step = -0.05;
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
        lastRb = gamepad1.right_bumper;
        lastLb = gamepad1.left_bumper;

        if (step != 0) {
            positions[selected] = Math.max(0.0, Math.min(1.0, positions[selected] + step));
            started[selected] = true;
        }

        for (int i = 0; i < servos.length; i++) {
            if (started[i]) {
                servos[i].setPwmEnable();
                servos[i].setPosition(positions[i]);
            }
        }

        telemetry.addData("Moving", labels[selected]);
        for (int i = 0; i < servos.length; i++) {
            telemetry.addData(labels[i] + " position", "%.2f%s",
                    positions[i], started[i] ? "" : " (limp)");
        }
        telemetry.update();
    }
}