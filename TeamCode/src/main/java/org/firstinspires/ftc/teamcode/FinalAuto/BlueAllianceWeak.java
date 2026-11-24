package org.firstinspires.ftc.teamcode.FinalAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * BLUE ALLIANCE WEAK
 *
 * Exactly the routine in RedAllianceWeak, with every pose turned half a turn around the middle
 * of the field (see AutoBase and BlueAllianceStrong). CHECK BEFORE A MATCH: on the blue start
 * position, after INIT, the Driver Station should read about x 88, y 136, heading 270.
 */
@Autonomous(name = "BlueAllianceWeak", group = "Autonomous", preselectTeleOp = "MAIN TELEOP")
public class BlueAllianceWeak extends RedAllianceWeak {

    @Override
    protected boolean isBlueAlliance() {
        return true;
    }
}
