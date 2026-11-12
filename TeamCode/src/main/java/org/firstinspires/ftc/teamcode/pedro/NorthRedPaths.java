package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class NorthRedPaths {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Made public so BioBuzzAuto can access robotPaths.start
    public final Pose start = poseFactory.of(59.1248, 9.4931, 90);

    private final Pose tip1 = poseFactory.of(59.1775, 13.7721, 89.2952);
    private final Pose getflower = poseFactory.of(9.1224, 43.9031, 175.1566);
    private final Pose getflowerControl1 = poseFactory.of(97.301, 36.4313, 0);
    private final Pose shootStart = poseFactory.of(9.1224, 43.9031, 156);
    private final Pose shoot = poseFactory.of(56.4466, 118.2225, 90);
    private final Pose shootControl1 = poseFactory.of(18.5318, 123.955, 0);
    private final Pose shootControl2 = poseFactory.of(60.9075, 131.1539, 0);
    private final Pose strafetoflowerStart = poseFactory.of(56.4466, 118.2225, 90);
    private final Pose strafetoflower = poseFactory.of(46.279, 118.122, 90);
    private final Pose instakeflowerStart = poseFactory.of(46.279, 118.122, 90);
    private final Pose instakeflower = poseFactory.of(46.4709, 135.3386, 90);
    private final Pose tip2Start = poseFactory.of(46.4709, 135.3386, 0);
    private final Pose tip2 = poseFactory.of(55.0894, 108.4826, 270);
    private final Pose intakeballsStart = poseFactory.of(55.0894, 108.4826, 0);
    private final Pose intakeballs = poseFactory.of(9.7989, 7.7858, 230);
    private final Pose intakeballsControl1 = poseFactory.of(13.0205, 93.2083, 0);
    private final Pose shootagainStart = poseFactory.of(9.7989, 7.7858, 0);
    private final Pose shootagain = poseFactory.of(58.5209, 12.5717, 90);
    private final Pose park = poseFactory.of(11.3404, 99.3139, -108.7215);
    private final Pose parkControl1 = poseFactory.of(2.6281, 73.6061, 0);

    public Path tip1() {
        return com.pedropathing.api.Paths.line(start, tip1).tangent();
    }

    public Path getflower() {
        return com.pedropathing.api.Paths.curve(tip1, getflowerControl1, getflower).tangent();
    }

    public Path shoot() {
        return com.pedropathing.api.Paths.curve(shootStart, shootControl1, shootControl2, shoot).linear(shootStart, shoot);
    }

    public Path strafetoflower() {
        return com.pedropathing.api.Paths.line(strafetoflowerStart, strafetoflower).linear(strafetoflowerStart, strafetoflower);
    }

    public Path instakeflower() {
        return com.pedropathing.api.Paths.line(instakeflowerStart, instakeflower).linear(instakeflowerStart, instakeflower);
    }

    public Path tip2() {
        return com.pedropathing.api.Paths.line(tip2Start, tip2).linear(tip2Start, tip2);
    }

    public Path intakeballs() {
        return com.pedropathing.api.Paths.curve(intakeballsStart, intakeballsControl1, intakeballs).linear(intakeballsStart, intakeballs);
    }

    public Path shootagain() {
        return com.pedropathing.api.Paths.line(shootagainStart, shootagain).linear(shootagainStart, shootagain);
    }

    public Path park() {
        return com.pedropathing.api.Paths.curve(shootagain, parkControl1, park).reverseTangent();
    }
}