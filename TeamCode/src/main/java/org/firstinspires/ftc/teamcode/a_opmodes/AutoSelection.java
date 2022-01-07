package org.firstinspires.ftc.teamcode.a_opmodes;

import android.provider.Settings;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.a_opmodes.teleop.BaseOpMode;

public class AutoSelection extends BaseOpMode {

    @Override
    protected void subInit() {
        telemetry.addData("Controls", "X: carousel, Y:outtakeCube, B:parking, D-Pad: cycles");
    }

    @Override
    protected void subLoop() {

        telemetry.addData("Carousel", GlobalConfig.carousel);
        telemetry.addData("OuttakeCube", GlobalConfig.outtakeCube);
        telemetry.addData("Cycles", GlobalConfig.cycles);
        telemetry.addData("Parking", GlobalConfig.parking);
        telemetry.update();

        if (gamepad1.x)
            GlobalConfig.carousel = !GlobalConfig.carousel;
        else if (gamepad1.y)
            GlobalConfig.outtakeCube = !GlobalConfig.outtakeCube;
        else if (gamepad1.b)
            GlobalConfig.parking = !GlobalConfig.parking;
        else if (gamepad1.dpad_up)
            GlobalConfig.cycles += 1;
        else if (gamepad1.dpad_down)
            GlobalConfig.cycles -= 1;
    }
}
