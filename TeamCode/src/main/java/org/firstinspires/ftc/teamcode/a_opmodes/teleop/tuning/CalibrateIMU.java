package org.firstinspires.ftc.teamcode.a_opmodes.teleop.tuning;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.a_opmodes.teleop.BaseOpMode;
import org.firstinspires.ftc.teamcode.c_drive.RRMecanumDrive;

@TeleOp(name = "IMU calibration Opmode", group = "Utility" )
public class CalibrateIMU extends BaseOpMode {

    public BNO055IMU imu00;
    public BNO055IMU imu01;

    public boolean centricity = false;
    public double fieldCentricOffset0 = 0.0;
    public double fieldCentricOffset1 = 0.0;

    public static double SLOW_MODE_PERCENT = 0.4;

    @Override
    public void subInit() {
        this.imu00 = bot.imu0;
        this.imu01 = bot.imu1;

        telemetry.addLine("Init Done!");
        telemetry.update();
    }

    @Override
    public void subLoop() {

        drive();

        telemetry.addLine("First IMU axes:");
        telemetry.addData("IMU first axis", imu00.getAngularOrientation().firstAngle);
        telemetry.addData("IMU second axis", imu00.getAngularOrientation().secondAngle);
        telemetry.addData("IMU third axis", imu00.getAngularOrientation().thirdAngle);
        telemetry.addData("IMU axes by internal method", imu00.getAngularOrientation());
        telemetry.addLine("\n\nWOAH SECOND IMU!?!?!?\n");
        telemetry.addLine("Second IMU axes:");
        telemetry.addData("IMU first axis", imu01.getAngularOrientation().firstAngle);
        telemetry.addData("IMU second axis", imu01.getAngularOrientation().secondAngle);
        telemetry.addData("IMU third axis", imu01.getAngularOrientation().thirdAngle);
        telemetry.addData("IMU axes by internal method", imu01.getAngularOrientation());
    }

    private void drive(){//Driving ===================================================================================
        final double gyroAngle0 =
                bot.imu0.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).firstAngle
                        - fieldCentricOffset0;
        final double gyroAngle1 = (bot.imu1 != null) ?
                bot.imu1.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).firstAngle
                        - fieldCentricOffset1
                : gyroAngle0;
        final double avgGyroAngle = (gyroAngle0 + gyroAngle1)/2;

        Vector2d driveVector = new Vector2d(gamepadEx1.getLeftX(), gamepadEx1.getLeftY()),
                turnVector = new Vector2d(
                        gamepadEx1.getRightX() * Math.abs(gamepadEx1.getRightX()),
                        0);
        if (bot.roadRunner.mode == RRMecanumDrive.Mode.IDLE) {

            boolean dpadPressed = (gamepadEx1.getButton(GamepadKeys.Button.DPAD_DOWN) || gamepadEx1.getButton(GamepadKeys.Button.DPAD_UP)
                    || gamepadEx1.getButton(GamepadKeys.Button.DPAD_LEFT) || gamepadEx1.getButton(GamepadKeys.Button.DPAD_RIGHT));
            boolean buttonPressed = (gamepadEx1.getButton(GamepadKeys.Button.X) || gamepadEx1.getButton(GamepadKeys.Button.B));

            double forwardSpeed = (gamepadEx1.getButton(GamepadKeys.Button.DPAD_LEFT) || gamepadEx1.getButton(GamepadKeys.Button.DPAD_RIGHT)) ? (gamepadEx1.getButton(GamepadKeys.Button.DPAD_RIGHT) ? 1 : -1) : 0;
            double strafeSpeed = (gamepadEx1.getButton(GamepadKeys.Button.DPAD_DOWN) || gamepadEx1.getButton(GamepadKeys.Button.DPAD_UP)) ? (gamepadEx1.getButton(GamepadKeys.Button.DPAD_UP) ? 1 : -1) : 0;
            double turnSpeed = (gamepadEx1.getButton(GamepadKeys.Button.X) || gamepadEx1.getButton(GamepadKeys.Button.B)) ? (gamepadEx1.getButton(GamepadKeys.Button.B) ? 1 : -1) : 0;

            if (centricity) {//epic java syntax
                bot.drive.driveFieldCentric(
                        driveVector.getY() * driveSpeed,
                        driveVector.getX() * -driveSpeed,
                        turnVector.getX() * driveSpeed,
                        avgGyroAngle);
            }
            else if (dpadPressed || buttonPressed) {
                driveSpeed *= SLOW_MODE_PERCENT;
                bot.drive.driveRobotCentric(
                        strafeSpeed * SLOW_MODE_PERCENT * driveSpeed,
                        forwardSpeed * -SLOW_MODE_PERCENT * driveSpeed,
                        turnSpeed * SLOW_MODE_PERCENT * driveSpeed
                );
            }

            else {
                bot.drive.driveRobotCentric(
                        driveVector.getY() * driveSpeed,
                        driveVector.getX() * -driveSpeed,
                        turnVector.getX() * driveSpeed
                );
            }

        }
        if (gamepadEx1.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
            fieldCentricOffset0 = bot.imu0.getAngularOrientation()
                    .toAngleUnit(AngleUnit.DEGREES).firstAngle;
            fieldCentricOffset1 = bot.imu1.getAngularOrientation()
                    .toAngleUnit(AngleUnit.DEGREES).firstAngle;
        }
        if (gamepadEx1.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)){
            centricity = !centricity;
        }

    }

}
