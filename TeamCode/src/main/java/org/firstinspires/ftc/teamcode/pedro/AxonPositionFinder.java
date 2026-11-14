package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/*
 * Axon Position Finder
 * Finds the numbers for the SETTINGS block in AxonTeleOp.
 *
 *   A / B / X       : choose what you're adjusting (A = RGB light colour, B = flower arm, X = stopper)
 *   D-pad Up / Down : move 0.01
 *   Bumpers         : move 0.05
 *
 * RGB light: step until the light is pure RED, write the number down (RGB_RED),
 *            then step until it is pure GREEN (RGB_GREEN).
 * Nothing moves or lights up until you press a D-pad or bumper.
 * Stopper: step until it fully BLOCKS the balls (just short of the hard stop) = STOPPER_CLOSED_POS,
 *          then until it's fully OUT OF THE WAY (just short of the other stop) = STOPPER_OPEN_POS.
 */
@TeleOp(name = "Axon Position Finder", group = "Test")
public class AxonPositionFinder extends OpMode {

    private ServoImplEx[] servos;
    private final String[] labels = {"RGB LIGHT", "FLOWER ARM", "STOPPER"};
    private final double[] positions = {AxonTeleOp.RGB_RED, AxonTeleOp.ARM_UP, 0.50};
    private final boolean[] started = {false, false, false};
    private int selected = 0;

    private boolean lastUp, lastDown, lastRb, lastLb;

    @Override
    public void init() {
        servos = new ServoImplEx[]{
                hardwareMap.tryGet(ServoImplEx.class, AxonTeleOp.RGB_NAME),   // null if not configured
                hardwareMap.get(ServoImplEx.class, AxonTeleOp.FLOWER_ARM_NAME),
                hardwareMap.tryGet(ServoImplEx.class, AxonTeleOp.STOPPER_NAME)   // null if not configured
        };
        for (ServoImplEx s : servos) {
            if (s == null) continue;
            s.setPwmRange(new PwmControl.PwmRange(500, 2500));
            s.setPwmDisable();
        }
    }

    @Override
    public void loop() {
        if (gamepad1.a) selected = 0;
        if (gamepad1.b) selected = 1;
        if (gamepad1.x) selected = 2;

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
            if (started[i] && servos[i] != null) {
                servos[i].setPwmEnable();
                servos[i].setPosition(positions[i]);
            }
        }

        telemetry.addData("Adjusting", labels[selected] + "   (A = light, B = flower arm, X = stopper)");
        for (int i = 0; i < servos.length; i++) {
            if (servos[i] == null) {
                telemetry.addData(labels[i], "NOT CONFIGURED");
                continue;
            }
            telemetry.addData(labels[i] + " position", "%.3f%s",
                    positions[i], started[i] ? "" : " (off)");
        }
        telemetry.update();
    }
}
