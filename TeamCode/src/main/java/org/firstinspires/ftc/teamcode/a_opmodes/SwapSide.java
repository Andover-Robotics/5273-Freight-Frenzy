package org.firstinspires.ftc.teamcode.a_opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.b_hardware.Bot;
@Disabled
@Autonomous(name = "Swap Side", group = "Competition")
public class SwapSide extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
            waitForStart();
            GlobalConfig.side = GlobalConfig.side == GlobalConfig.Side.DEPOT ? GlobalConfig.Side.WAREHOUSE : GlobalConfig.Side.DEPOT;
            Bot.instance = null;
            telemetry.addLine(GlobalConfig.side.toString() + " is now sus");
            telemetry.update();
        }
}
