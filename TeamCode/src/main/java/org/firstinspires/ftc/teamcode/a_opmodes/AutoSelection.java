package org.firstinspires.ftc.teamcode.a_opmodes;

import android.opengl.GLES10;
import android.provider.Settings;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.a_opmodes.teleop.BaseOpMode;

@Autonomous(name = "Autonomous Selection", group = "Competition")

public class AutoSelection extends BaseOpMode {
    //todo clean up opmodes, so the selection screen is not cluttered
    @Override
    protected void subInit() {
        telemetry.addData("Controls", "B: carousel, Y: outtakeCube, X: parking, A: alliance, Bumpers: side, D-Pad: cycles");
    }

    @Override
    protected void subLoop() {

        telemetry.addData("Carousel", GlobalConfig.carousel);
        telemetry.addData("OuttakeCube", GlobalConfig.outtakeCube);
        telemetry.addData("Cycles", GlobalConfig.cycles);
        telemetry.addData("Parking", GlobalConfig.parking);
        telemetry.addData("Alliance", GlobalConfig.alliance);
        telemetry.addData("Side", GlobalConfig.side);
        telemetry.update();

        if (gamepadEx1.wasJustReleased(GamepadKeys.Button.B))
            GlobalConfig.carousel = !GlobalConfig.carousel;
        else if (gamepadEx1.wasJustReleased(GamepadKeys.Button.Y))
            GlobalConfig.outtakeCube = !GlobalConfig.outtakeCube;
        else if (gamepadEx1.wasJustReleased(GamepadKeys.Button.X))
            GlobalConfig.parking = !GlobalConfig.parking;
        else if (gamepadEx1.wasJustReleased(GamepadKeys.Button.DPAD_UP))
            GlobalConfig.cycles += 1;
        else if (gamepadEx1.wasJustReleased(GamepadKeys.Button.DPAD_DOWN))
            GlobalConfig.cycles -= 1;
        else if (gamepadEx1.wasJustReleased(GamepadKeys.Button.A)) {
            if (GlobalConfig.alliance == GlobalConfig.Alliance.BLUE)
                GlobalConfig.alliance = GlobalConfig.Alliance.RED;
            else if (GlobalConfig.alliance == GlobalConfig.Alliance.RED)
                GlobalConfig.alliance = GlobalConfig.Alliance.BLUE;
        }
        else if (gamepadEx1.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER)){
            if (GlobalConfig.alliance == GlobalConfig.Alliance.RED)
                GlobalConfig.side = GlobalConfig.Side.DEPOT;
            else if (GlobalConfig.alliance == GlobalConfig.Alliance.BLUE)
                GlobalConfig.side = GlobalConfig.Side.WAREHOUSE;
        }
        else if (gamepadEx1.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)){
            if (GlobalConfig.alliance == GlobalConfig.Alliance.RED)
                GlobalConfig.side = GlobalConfig.Side.WAREHOUSE;
            else if (GlobalConfig.alliance == GlobalConfig.Alliance.BLUE)
                GlobalConfig.side = GlobalConfig.Side.DEPOT;
        }
    }
}
