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
    private DcMotor leftouttake;
    private DcMotor rightouttake;

    @Override
    public void init() {
        drivetrain = new Mecanum(hardwareMap, Constants.drivetrainConfig);
        intake = hardwareMap.get(DcMotor.class, "intake");
        leftouttake = hardwareMap.get(DcMotor.class,"leftouttake" );
        rightouttake = hardwareMap.get(DcMotor.class,"rightouttake" );


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

        if (gamepad2.square) {
            intake.setPower(1.0);
        } else if (gamepad2.circle) {
            intake.setPower(-1.0);
        } else {
            intake.setPower(0.0);
        }

        if (gamepad2.right_trigger > 0.1) {
            rightouttake.setPower(1.0);
        } else if (gamepad2.x) {
            rightouttake.setPower(-1.0);
        } else {
            rightouttake.setPower(0.0);
        }

        if (gamepad2.left_trigger > 0.1) {
            leftouttake.setPower(1.0);
        } else if (gamepad2.triangle) {
            leftouttake.setPower(-1.0);
        } else {
            leftouttake.setPower(0.0);
        }

    }
}

