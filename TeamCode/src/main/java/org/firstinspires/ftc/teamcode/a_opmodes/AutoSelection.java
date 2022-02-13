package org.firstinspires.ftc.teamcode.a_opmodes;

import android.provider.Settings;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.a_opmodes.teleop.BaseOpMode;

@Autonomous(name="Autonomous Selection", group = "Competition")

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

        if (gamepadEx1.wasJustReleased(GamepadKeys.Button.B))
            GlobalConfig.carousel = !GlobalConfig.carousel;
        else if (gamepadEx1.wasJustReleased(GamepadKeys.Button.Y))
            GlobalConfig.outtakeCube = !GlobalConfig.outtakeCube;
        else if (gamepadEx1.wasJustReleased(GamepadKeys.Button.B))
            GlobalConfig.parking = !GlobalConfig.parking;
        else if (gamepadEx1.wasJustReleased(GamepadKeys.Button.DPAD_UP))
            GlobalConfig.cycles += 1;
        else if (gamepadEx1.wasJustReleased(GamepadKeys.Button.DPAD_DOWN))
            GlobalConfig.cycles -= 1;
    }
}
