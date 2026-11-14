package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
 * "zack blue south" - Zack's auto: shoot the 4 preloaded Pollen, then park in the BLUE LOADING ZONE.
 *
 * This file only holds WHERE things are on the field. What the robot does is in ZackAuto.java.
 *
 * WHERE TO PUT THE ROBOT
 *   Back flat against the AUDIENCE wall, on the BLUE side, robot centre 10 in from the centre
 *   seam (60.75 in from the inside of the blue wall). Front and Limelight facing the HIVE.
 *   There is a FLOWER on this wall, on the next tile seam. Rule G304: at the start the robot may
 *   NOT touch a FLOWER and must be completely on the blue half. This spot leaves a gap on both
 *   sides for a robot up to 18 in wide.
 *   This is "zack red north" turned half a turn around the middle of the field.
 *
 * WHEN IT CAN SCORE - READ THIS
 *   A CELL can only be scored from ITS OWN side of the HIVE (its opening faces outward), and only
 *   while it is the one tilted UP. Every match starts with the BLUE cell on the REAR side up and
 *   the blue cell on THIS side DOWN (Event Field Setup Guide).
 *   So at the start of a real match this auto has nothing to shoot at: it only scores after the
 *   blue HIVE has been tipped once (by a partner). For the start of a match use "zack blue north".
 *   There is no FLOWER trip here on purpose: until that cell comes up, more pollen can't score.
 *   Not run on the robot yet (Nov 13).
 *
 * Field numbers: x = inches from the inside of the RED wall, y = inches from the inside of the
 * AUDIENCE wall (the floor is 141.5 in wall to wall, not 144),
 * heading 90 = facing away from the audience, 270 = facing the audience.
 */
@Autonomous(name = "zack blue south", group = "Autonomous")
public class ZackBlueSouth extends ZackAuto {
    @Override
    protected void settings() {
        title = "zack blue south";
        alliance = HiveTarget.Alliance.BLUE;
        start(80.75, 9.3638, 90);       // back on the audience wall, 10 in from the centre seam, clear of the FLOWER at x = 94.75
        shoot(80.75, 17.9775, 90);      // 8.6 in off the wall
        park(131.0, 34.75);             // middle of the blue LOADING ZONE (tile F2: blue wall, 2nd row from the audience)
    }
}