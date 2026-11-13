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

@Autonomous(name = "SouthRedAutoPaths", group = "Autonomous")
public class SouthRedAutoPaths extends LinearOpMode {

    private Follower follower;
    private Flywheel flywheel;
    private Stopper stopper;
    private Battery battery;
    private DcMotorEx intake;
    private ServoImplEx flowerArm;
    private boolean flywheelRunning = false;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(59.1248, 9.4931, 90);
    private final Pose tip1 = poseFactory.of(59.1775, 13.7721, 89.2952);
    private final Pose getflower = poseFactory.of(9.1224, 43.9031, 175.1566);
    private final Pose getflowerControl1 = poseFactory.of(97.301, 36.4313, 0);
    private final Pose shootPose = poseFactory.of(56.4466, 118.2225, 90);
    private final Pose shootControl1 = poseFactory.of(18.5318, 123.955, 0);
    private final Pose shootControl2 = poseFactory.of(60.9075, 131.1539, 0);
    private final Pose strafetoflower = poseFactory.of(46.279, 118.122, 90);
    private final Pose instakeflower = poseFactory.of(46.4709, 135.3386, 90);
    private final Pose tip2 = poseFactory.of(55.0894, 108.4826, 270);
    private final Pose intakeballs = poseFactory.of(9.7989, 7.7858, 230);
    private final Pose intakeballsControl1 = poseFactory.of(13.0205, 93.2083, 0);
    private final Pose shootagain = poseFactory.of(58.5209, 12.5717, 90);
    private final Pose park = poseFactory.of(11.3404, 99.3139, -108.7215);
    private final Pose parkControl1 = poseFactory.of(2.6281, 73.6061, 0);

    public Command autoRoutine() {
        return sequential(
                follow(follower, tip1Path()),

                // Get flower
                parallel(
                        follow(follower, getflowerPath()),
                        sequential(
                                instant(() -> flowerArm.setPosition(0.240)),
                                instant(() -> intake.setPower(1.0))
                        )
                ),
                instant(() -> intake.setPower(0.0)),

                // Shoot
                parallel(
                        follow(follower, shootPath()),
                        instant(() -> flywheelRunning = true)
                ),
                instant(() -> sleep(800)),
                instant(() -> stopper.update(true)),
                instant(() -> sleep(500)),
                instant(() -> stopper.startClosed()),
                instant(() -> flywheelRunning = false),
                instant(() -> flowerArm.setPosition(0.810)),

                follow(follower, strafetoflowerPath()),

                // Intake flower
                parallel(
                        follow(follower, instakeflowerPath()),
                        sequential(
                                instant(() -> flowerArm.setPosition(0.240)),
                                instant(() -> intake.setPower(1.0))
                        )
                ),
                instant(() -> intake.setPower(0.0)),

                follow(follower, tip2Path()),

                // Intake balls
                parallel(
                        follow(follower, intakeballsPath()),
                        sequential(
                                instant(() -> flowerArm.setPosition(0.240)),
                                instant(() -> intake.setPower(1.0))
                        )
                ),
                instant(() -> intake.setPower(0.0)),

                // Shoot again
                parallel(
                        follow(follower, shootagainPath()),
                        instant(() -> flywheelRunning = true)
                ),
                instant(() -> sleep(800)),
                instant(() -> stopper.update(true)),
                instant(() -> sleep(500)),
                instant(() -> stopper.startClosed()),
                instant(() -> flywheelRunning = false),
                instant(() -> flowerArm.setPosition(0.810)),

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
        flowerArm.setPosition(0.810);

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

    public Path tip1Path() { return Paths.line(start, tip1).tangent(); }
    public Path getflowerPath() { return Paths.curve(tip1, getflowerControl1, getflower).tangent(); }
    public Path shootPath() { return Paths.curve(getflower, shootControl1, shootControl2, shootPose).linear(getflower, shootPose); }
    public Path strafetoflowerPath() { return Paths.line(shootPose, strafetoflower).linear(shootPose, strafetoflower); }
    public Path instakeflowerPath() { return Paths.line(strafetoflower, instakeflower).linear(strafetoflower, instakeflower); }
    public Path tip2Path() { return Paths.line(instakeflower, tip2).linear(instakeflower, tip2); }
    public Path intakeballsPath() { return Paths.curve(tip2, intakeballsControl1, intakeballs).linear(tip2, intakeballs); }
    public Path shootagainPath() { return Paths.line(intakeballs, shootagain).linear(intakeballs, shootagain); }
    public Path parkPath() { return Paths.curve(shootagain, parkControl1, park).reverseTangent(); }
}