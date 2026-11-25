package org.firstinspires.ftc.teamcode.FinalAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * The blue version of RedAllianceStrong. There is deliberately nothing else in here: AutoBase turns
 * every red pose half a turn about the middle of the field, so a fix to the red auto is a fix
 * to this one too.
 *
 * Check before trusting it: on the blue start spot, after INIT, the Driver Station should read
 * about x 88, y 136, heading 90.
 */
@Autonomous(name = "Blue Alliance Strong", group = "Autonomous", preselectTeleOp = "MAIN TELEOP")
public class BlueAllianceStrong extends RedAllianceStrong {

    @Override
    protected boolean isBlueAlliance() {
        return true;
    }
}
