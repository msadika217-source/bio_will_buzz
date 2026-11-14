package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
 * "zack red north" - Zack's auto: shoot the 4 preloaded Pollen, then park in the RED LOADING ZONE.
 *
 * This file only holds WHERE things are on the field. What the robot does is in ZackAuto.java.
 *
 * WHERE TO PUT THE ROBOT
 *   Back flat against the REAR wall (the one far from the audience), on the RED side, robot centre
 *   10 in from the centre seam (60.75 in from the inside of the red wall). Front and Limelight
 *   facing the HIVE.
 *   There is a FLOWER on this wall, on the next tile seam. Rule G304: at the start the robot may
 *   NOT touch a FLOWER and must be completely on the red half. This spot leaves a gap on both
 *   sides for a robot up to 18 in wide.
 *
 * WHEN IT CAN SCORE - READ THIS
 *   A CELL can only be scored from ITS OWN side of the HIVE (its opening faces outward), and only
 *   while it is the one tilted UP. Every match starts with the RED cell on the AUDIENCE side up
 *   and the red cell on THIS side DOWN (Event Field Setup Guide).
 *   So at the start of a real match this auto has nothing to shoot at: it only scores after the
 *   red HIVE has been tipped once (by a partner). For the start of a match use "zack red south".
 *   There is no FLOWER trip here on purpose: until that cell comes up, more pollen can't score.
 *   Tested on the practice field Nov 13 with the rear red cell up: all 4 preloads went in.
 *
 * Field numbers: x = inches from the inside of the RED wall, y = inches from the inside of the
 * AUDIENCE wall (the floor is 141.5 in wall to wall, not 144),
 * heading 90 = facing away from the audience, 270 = facing the audience.
 */
@Autonomous(name = "zack red north", group = "Autonomous")
public class ZackRedNorth extends ZackAuto {
    @Override
    protected void settings() {
        title = "zack red north";
        alliance = HiveTarget.Alliance.RED;
        start(60.75, 132.1362, 270);    // back on the rear wall, 10 in from the centre seam, clear of the FLOWER at x = 46.75
        shoot(60.75, 123.5225, 270);    // 8.6 in off the wall
        park(10.5, 106.75);             // middle of the red LOADING ZONE (tile A5: red wall, 5th row from the audience)
    }
}