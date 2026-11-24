package org.firstinspires.ftc.teamcode.FinalAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueAllianceWeak", group = "Autonomous", preselectTeleOp = "MAIN TELEOP")
public class BlueAllianceWeak extends RedAllianceWeak {

    @Override
    protected boolean isBlueAlliance() {
        return true;
    }
}
