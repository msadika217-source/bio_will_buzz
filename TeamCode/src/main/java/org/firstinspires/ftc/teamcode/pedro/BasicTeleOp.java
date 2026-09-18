package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.drivetrain.DrivePowers;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Basic TeleOp", group = "Drive")
public class BasicTeleOp extends OpMode {

    private Mecanum drivetrain;
    private DcMotor intake;

    @Override
    public void init() {
        drivetrain = new Mecanum(hardwareMap, Constants.drivetrainConfig);
        intake = hardwareMap.get(DcMotor.class, "intake");
    }

    @Override
    public void loop() {
        drivetrain.drive(
                new DrivePowers(
                        -gamepad1.left_stick_y, // Forward/backward
                        gamepad1.left_stick_x,  // Strafe left/right
                        gamepad1.right_stick_x  // Turn left/right
                ),
                true // manual drive mode
        );

        if (gamepad1.right_trigger > 0.1) {
            intake.setPower(1.0);
        } else if (gamepad1.left_trigger > 0.1) {
            intake.setPower(-1.0);
        } else {
            intake.setPower(0.0);
        }
    }
}