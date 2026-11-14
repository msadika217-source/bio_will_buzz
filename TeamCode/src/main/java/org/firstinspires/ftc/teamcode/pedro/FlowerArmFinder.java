package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/*
 * Flower Arm Finder - finds ARM_UP and ARM_DOWN for the NEW flower intake (Axon MAX).
 * Uses the same servo name ("flowerArm") and the same 500-2500 range as zack code, so the
 * numbers you find here go straight into AxonTeleOp.
 *
 * The servo stays LIMP (no power) until you press a move button. It then goes to 0.500 first,
 * so have the arm roughly in the middle of its travel before you press anything.
 *
 *   D-pad UP / DOWN       : +0.01 / -0.01
 *   Bumpers  R1 / L1      : +0.05 / -0.05
 *   D-pad RIGHT / LEFT    : +0.002 / -0.002   (fine steps near a hard stop)
 *
 *   TRIANGLE : save current position as UP   (stowed, inside 18 in)
 *   CROSS    : save current position as DOWN (ramp under the Flower)
 *   SQUARE   : test - jump between saved UP and DOWN at full speed
 *   CIRCLE   : go LIMP right away (if it's straining against something)
 *
 * Stop each position just BEFORE the arm hits a hard stop. If the servo buzzes or hums
 * while sitting still, it's pushing against something - back it off 0.01-0.02.
 */
@TeleOp(name = "Flower Arm Finder", group = "Test")
public class FlowerArmFinder extends OpMode {

    private ServoImplEx arm;
    private double position = 0.500;
    private boolean powered = false;

    private double savedUp = -1;
    private double savedDown = -1;
    private boolean testAtUp = true;

    private boolean lastUp, lastDown, lastRight, lastLeft, lastRb, lastLb;
    private boolean lastTriangle, lastCross, lastSquare;

    @Override
    public void init() {
        arm = hardwareMap.get(ServoImplEx.class, AxonTeleOp.FLOWER_ARM_NAME);
        arm.setPwmRange(new PwmControl.PwmRange(500, 2500));
        arm.setPwmDisable();   // limp until you press something
        telemetry.addLine("Put the arm roughly in the MIDDLE of its travel, then press PLAY.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // ---- Step the position ----
        double step = 0;
        if (gamepad1.dpad_up && !lastUp)          step = 0.01;
        if (gamepad1.dpad_down && !lastDown)      step = -0.01;
        if (gamepad1.dpad_right && !lastRight)    step = 0.002;
        if (gamepad1.dpad_left && !lastLeft)      step = -0.002;
        if (gamepad1.right_bumper && !lastRb)     step = 0.05;
        if (gamepad1.left_bumper && !lastLb)      step = -0.05;
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
        lastRight = gamepad1.dpad_right;
        lastLeft = gamepad1.dpad_left;
        lastRb = gamepad1.right_bumper;
        lastLb = gamepad1.left_bumper;

        if (step != 0) {
            if (powered) position = clamp(position + step);   // first press just powers on at 0.500
            powered = true;
        }

        // ---- Save UP / DOWN ----
        if (gamepad1.triangle && !lastTriangle && powered) savedUp = position;
        if (gamepad1.cross && !lastCross && powered) savedDown = position;
        lastTriangle = gamepad1.triangle;
        lastCross = gamepad1.cross;

        // ---- Test: jump between saved UP and DOWN ----
        if (gamepad1.square && !lastSquare && savedUp >= 0 && savedDown >= 0) {
            testAtUp = !testAtUp;
            position = testAtUp ? savedUp : savedDown;
            powered = true;
        }
        lastSquare = gamepad1.square;

        // ---- Emergency limp ----
        if (gamepad1.circle) powered = false;

        // ---- Drive the servo ----
        if (powered) {
            arm.setPwmEnable();
            arm.setPosition(position);
        } else {
            arm.setPwmDisable();
        }

        // ---- Screen ----
        telemetry.addData("Flower arm position", "%.3f   %s", position, powered ? "" : "(LIMP)");
        telemetry.addData("Saved UP   (Triangle)", savedUp >= 0 ? String.format("%.3f", savedUp) : "-");
        telemetry.addData("Saved DOWN (Cross)", savedDown >= 0 ? String.format("%.3f", savedDown) : "-");
        telemetry.addLine("D-pad U/D +-0.01 | D-pad R/L +-0.002 | Bumpers +-0.05");
        telemetry.addLine("Square = test UP<->DOWN | Circle = go limp");
        telemetry.update();
    }

    @Override
    public void stop() {
        arm.setPwmDisable();
    }

    private static double clamp(double v) {
        return Math.max(0.0, Math.min(1.0, v));
    }
}
