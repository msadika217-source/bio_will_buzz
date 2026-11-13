package org.firstinspires.ftc.teamcode.pedro;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Autonomous(name = "NorthBlueAuto Path", group = "Autonomous")
public class NorthBlueAutoPaths extends LinearOpMode {

    private Follower follower;
    private Flywheel flywheel;
    private Stopper stopper;
    private Battery battery;
    private DcMotorEx intake;
    private ServoImplEx flowerArm;
    private boolean flywheelRunning = false;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(80.857, 134.9721, 270);
    private final Pose shootPose = poseFactory.of(81.5288, 125.5187, 280);
    private final Pose intakegarden = poseFactory.of(139.243, 130.9996, 90);
    private final Pose intakegardenControl1 = poseFactory.of(86.2192, 127.7115, 0);
    private final Pose shoot_2 = poseFactory.of(78.1444, 14.8888, -9);
    private final Pose shoot_2Control1 = poseFactory.of(107.9039, 38.2125, 0);
    private final Pose intakeflower = poseFactory.of(95.4816, 12.4229, 90);
    private final Pose shoot_3 = poseFactory.of(88.2028, 21.1814, 90);
    private final Pose intakeflower_2 = poseFactory.of(132.3784, 95.0955, 0);
    private final Pose shoot_4 = poseFactory.of(82.4464, 109.1826, 270);
    private final Pose park = poseFactory.of(130.9354, 45.9169, 72.5006);
    private final Pose parkControl1 = poseFactory.of(141.5, 79.4247, 0);

    // Autonomous routine with mechanism integration
    public Command autoRoutine() {
        return sequential(
                // 1. Initial preload shot
                instant(() -> flywheelRunning = true),
                instant(() -> flowerArm.setPosition(0.240)), // ARM_DOWN
                instant(() -> sleep(800)),
                instant(() -> stopper.update(true)), // toggles/opens stopper via public method
                instant(() -> sleep(500)),
                instant(() -> stopper.startClosed()),
                instant(() -> flywheelRunning = false),
                instant(() -> flowerArm.setPosition(0.810)), // ARM_UP

                // 2. Intake garden
                parallel(
                        follow(follower, intakegardenPath()),
                        sequential(
                                instant(() -> flowerArm.setPosition(0.240)),
                                instant(() -> intake.setPower(1.0))
                        )
                ),
                instant(() -> intake.setPower(0.0)),

                // 3. Shoot 2
                parallel(
                        follow(follower, shoot2Path()),
                        instant(() -> flywheelRunning = true)
                ),
                instant(() -> sleep(800)),
                instant(() -> stopper.update(true)),
                instant(() -> sleep(500)),
                instant(() -> stopper.startClosed()),
                instant(() -> flywheelRunning = false),
                instant(() -> flowerArm.setPosition(0.810)),

                // 4. Intake flower
                parallel(
                        follow(follower, intakeflowerPath()),
                        sequential(
                                instant(() -> flowerArm.setPosition(0.240)),
                                instant(() -> intake.setPower(1.0))
                        )
                ),
                instant(() -> intake.setPower(0.0)),

                // 5. Shoot 3
                parallel(
                        follow(follower, shoot3Path()),
                        instant(() -> flywheelRunning = true)
                ),
                instant(() -> sleep(800)),
                instant(() -> stopper.update(true)),
                instant(() -> sleep(500)),
                instant(() -> stopper.startClosed()),
                instant(() -> flywheelRunning = false),
                instant(() -> flowerArm.setPosition(0.810)),

                // 6. Intake flower 2
                parallel(
                        follow(follower, intakeflower2Path()),
                        sequential(
                                instant(() -> flowerArm.setPosition(0.240)),
                                instant(() -> intake.setPower(1.0))
                        )
                ),
                instant(() -> intake.setPower(0.0)),

                // 7. Shoot 4
                parallel(
                        follow(follower, shoot4Path()),
                        instant(() -> flywheelRunning = true)
                ),
                instant(() -> sleep(800)),
                instant(() -> stopper.update(true)),
                instant(() -> sleep(500)),
                instant(() -> stopper.startClosed()),
                instant(() -> flywheelRunning = false),
                instant(() -> flowerArm.setPosition(0.810)),

                // 8. Park
                follow(follower, parkPath())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        battery = new Battery(hardwareMap);
        stopper = new Stopper(hardwareMap);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flowerArm = hardwareMap.get(ServoImplEx.class, AxonTeleOp.FLOWER_ARM_NAME);
        flowerArm.setPwmRange(new PwmControl.PwmRange(500, 2500));
        flowerArm.setPosition(0.810); // ARM_UP

        stopper.startClosed();
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            battery.update();
            flywheel.update(flywheelRunning, AxonTeleOp.SHOOT_RPM, battery);
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());
            telemetry.update();
        }
    }

    public Path shootPath() { return Paths.line(start, shootPose).linear(start, shootPose); }
    public Path intakegardenPath() { return Paths.curve(shootPose, intakegardenControl1, intakegarden).linear(shootPose, intakegarden); }
    public Path shoot2Path() { return Paths.curve(intakegarden, shoot_2Control1, shoot_2).linear(intakegarden, shoot_2); }
    public Path intakeflowerPath() { return Paths.line(shoot_2, intakeflower).linear(shoot_2, intakeflower); }
    public Path shoot3Path() { return Paths.line(intakeflower, shoot_3).linear(intakeflower, shoot_3); }
    public Path intakeflower2Path() { return Paths.line(shoot_3, intakeflower_2).linear(shoot_3, intakeflower_2); }
    public Path shoot4Path() { return Paths.line(intakeflower_2, shoot_4).linear(intakeflower_2, shoot_4); }
    public Path parkPath() { return Paths.curve(shoot_4, parkControl1, park).reverseTangent(); }
}