package org.firstinspires.ftc.teamcode.a_opmodes.teleop.tuning;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.a_opmodes.teleop.BaseOpMode;

@TeleOp(name = "IMU calibration Opmode", group = "Utility" )
public class CalibrateIMU extends BaseOpMode {

    public BNO055IMU imu;


    @Override
    public void subInit() {
        imu = bot.imu;

        telemetry.addLine("Init Done!");
        telemetry.update();
    }

    @Override
    public void subLoop() {
        telemetry.addLine("IMU axes");
        telemetry.addData("IMU first axis", imu.getAngularOrientation().firstAngle);
        telemetry.addData("IMU second axis", imu.getAngularOrientation().secondAngle);
        telemetry.addData("IMU third axis", imu.getAngularOrientation().thirdAngle);
        telemetry.addLine();
        telemetry.addData("IMU axes by internal method", imu.getAngularOrientation());
        telemetry.update();
    }

}
