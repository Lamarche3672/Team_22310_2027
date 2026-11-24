package org.firstinspires.ftc.teamcode.FinalAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * BLUE ALLIANCE STRONG
 *
 * Exactly the routine in RedAllianceStrong: every step, speed and time limit lives there, so a
 * change to Red is a change to Blue too. The only difference is where things are on the field:
 * AutoBase turns every red pose half a turn (180 degrees) around the middle of the field, so
 * (x, y, heading) becomes (144 - x, 144 - y, heading + 180).
 *
 * That layout comes from the game manual's text (red garden by the audience wall, blue garden in
 * the opposite corner by the rear wall), not from a measured field. CHECK BEFORE A MATCH: put the
 * robot on the blue start position and press INIT. The Driver Station shows where the robot
 * should be (about x 88, y 136, heading 90) and where the Pinpoint says it is.
 */
@Autonomous(name = "Blue Alliance Strong", group = "Autonomous", preselectTeleOp = "MAIN TELEOP")
public class BlueAllianceStrong extends RedAllianceStrong {

    @Override
    protected boolean isBlueAlliance() {
        return true;
    }
}
