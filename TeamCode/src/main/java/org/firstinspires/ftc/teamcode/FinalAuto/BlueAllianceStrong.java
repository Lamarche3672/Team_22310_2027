package org.firstinspires.ftc.teamcode.FinalAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Alliance Strong", group = "Autonomous", preselectTeleOp = "MAIN TELEOP")
public class BlueAllianceStrong extends RedAllianceStrong {

    @Override
    protected boolean isBlueAlliance() {
        return true;
    }
}
