package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.groups.Groups.sequential;

@Autonomous(name = "North Red", group = "Autonomous")
public class NorthRedBioBuzzAuto extends OpMode {
    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();
    private NorthRedPaths robotPaths;

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);

        // 1. Initialize paths first, then set the starting pose using robotPaths.start
        robotPaths = new NorthRedPaths();
        follower.setPose(robotPaths.start);

        telemetry.addData("Status", "Auto Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        follower.update();
    }

    @Override
    public void start() {
        schedule(sequential(
                follow(follower, robotPaths.tip1()),
                follow(follower, robotPaths.getflower()),
                follow(follower, robotPaths.shoot()),
                follow(follower, robotPaths.strafetoflower()),
                follow(follower, robotPaths.instakeflower()),
                follow(follower, robotPaths.tip2()),
                follow(follower, robotPaths.intakeballs()),
                follow(follower, robotPaths.shootagain()),
                follow(follower, robotPaths.park())
        ));
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getYHeading());
        telemetry.update();
    }
}