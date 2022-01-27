package org.firstinspires.ftc.teamcode.a_opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.c_drive.RRMecanumDrive;

@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends BaseOpMode {//required vars here
  private double cycle = 0;
  private double prevRead = 0;
  private boolean centricity = false;
  private final double TRIGGER_CONSTANT = 0.15;
  private final double SLOW_MODE_PERCENT = 0.4;
  private double fieldCentricOffset0 = 0.0;
  private double fieldCentricOffset1 = 0.0;

  //config? stuff here =========================================================================

  private double fieldCentricOffset = 0.0;

  //opmode vars here ==============================================================================================
  public void subInit() {
    driveSpeed = 1.0;
  }

  @Override
  public void subLoop() {
    //update stuff=================================================================================================
    cycle = 1.0/(time-prevRead);
    prevRead = time;
    timingScheduler.run();

    //Movement ==================================================================================================
    drive();

    //Sensors ===================================================================================================


    //Subsystem control =========================================================================================

    // intake controls
    if (gamepadEx1.isDown(Button.LEFT_BUMPER)){
      bot.intake.reverseLeft();
    }
    else if (gamepadEx1.getTrigger(Trigger.LEFT_TRIGGER) > TRIGGER_CONSTANT){
      bot.intake.runLeft();
      bot.outtake.openLeftFlap();
    }
    else {
      bot.intake.stopLeft();
      if(!bot.outtake.isAutoFlap()) {
        bot.outtake.closeLeftFlap();
      }
    }

    if (gamepadEx1.isDown(Button.RIGHT_BUMPER)) {
      bot.intake.reverseRight();
    }
    else if (gamepadEx1.getTrigger(Trigger.RIGHT_TRIGGER) > TRIGGER_CONSTANT) {
      bot.intake.runRight();
      bot.outtake.openRightFlap();
    }
    else {
      bot.intake.stopRight();
      if(!bot.outtake.isAutoFlap()) {
        bot.outtake.closeRightFlap();
      }
    }

    if(gamepadEx1.wasJustPressed(Button.DPAD_LEFT)) {
      bot.outtake.closeLeftFlap();
    }
    else if(gamepadEx1.wasJustPressed(Button.DPAD_RIGHT)) {
      bot.outtake.closeRightFlap();
    }
    else if(gamepadEx1.wasJustPressed(Button.DPAD_UP)) {
      bot.outtake.toggleAutoFlap();
    }



    if (gamepadEx2.isDown(Button.LEFT_BUMPER)) {
      bot.outtake.openLeftFlap();
    }

    else if (gamepadEx2.isDown(Button.RIGHT_BUMPER)) {
      bot.outtake.openRightFlap();
    }

    // all slides controls
    if(gamepadEx2.wasJustPressed(Button.LEFT_STICK_BUTTON)) {
      bot.outtake.goToCapstone();
    }
    else if(gamepadEx2.wasJustPressed(Button.DPAD_UP)) {
      bot.outtake.goToTopGoal();
      timingScheduler.defer(0.26,
              () -> { bot.outtake.flipBucket();
                timingScheduler.defer(0.5,
                        () -> {
                          bot.outtake.unFlipBucket();
                          bot.outtake.fullyRetract();
                        }
                );
              }
      );
    }
    else if(gamepadEx2.wasJustPressed(Button.DPAD_RIGHT)) {
      bot.outtake.goToLowGoal();
      timingScheduler.defer(0.06,
              () -> { bot.outtake.flipBucket();
                timingScheduler.defer(0.5,
                                () -> {
                                  bot.outtake.unFlipBucket();
                                  bot.outtake.fullyRetract();
                                }

                );
              }
      );
    }
    else if(gamepadEx2.wasJustPressed(Button.DPAD_LEFT)) {
      bot.outtake.goToMidGoal();
      timingScheduler.defer(0.16,
              () -> { bot.outtake.flipBucket();
                timingScheduler.defer(0.5,
                                () -> {
                                  bot.outtake.unFlipBucket();
                                  bot.outtake.fullyRetract();
                                }

                );
              }
      );
    }
    else if(gamepadEx2.wasJustPressed(Button.DPAD_DOWN)) {
      bot.outtake.fullyRetract();
    }

    if (gamepadEx2.wasJustPressed(Button.LEFT_BUMPER)){
      timingScheduler.clearAll();
    }

    // carousel controls
    if (gamepadEx2.wasJustPressed(Button.A)){
      bot.carousel.toggleBlue();
    }
    else if (gamepadEx2.wasJustPressed(Button.B)) {
      bot.carousel.toggleRed();
    }
    else if (gamepadEx2.wasJustPressed(Button.X)) {
      bot.carousel.stop();
    }

    CommandScheduler.getInstance().run();

    telemetry.addData("Centricity", centricity);
    telemetry.addData("cycle", cycle);
    telemetry.addData("x", bot.roadRunner.getPoseEstimate().getX());
    telemetry.addData("y", bot.roadRunner.getPoseEstimate().getY());
    telemetry.addData("heading", bot.roadRunner.getPoseEstimate().getHeading());
    telemetry.addData("driver left stick", "left X" + gamepadEx1.getLeftX() + ": " + gamepadEx1.getLeftY());
    telemetry.addLine("isFreightIn : " + bot.outtake.isFreightIn());
  }


  private void drive(){//Driving ===================================================================================

    final double gyroTolerance = 0.05;

    final double gyroAngle0 =
            bot.imu0.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).firstAngle
                    - fieldCentricOffset0;
    final double gyroAngle1 = (bot.imu1 != null) ?
            bot.imu1.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).firstAngle
                    - fieldCentricOffset1
            : gyroAngle0;
    final double avgGyroAngle = ((gyroAngle0 + gyroAngle1)/2);

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
                        (  Math.abs(avgGyroAngle - gyroAngle0) < gyroTolerance
                        || Math.abs(avgGyroAngle - gyroAngle1) < gyroTolerance) ?
                                Math.abs(gyroAngle0 - avgGyroAngle) <
                                        Math.abs(gyroAngle1 - avgGyroAngle) ?
                                        gyroAngle0 : gyroAngle1 : avgGyroAngle
        );
      }
      else if (dpadPressed || buttonPressed) {
        double tempDriveSpeed = driveSpeed *= SLOW_MODE_PERCENT;
        bot.drive.driveRobotCentric(
                strafeSpeed * tempDriveSpeed,
                forwardSpeed * -tempDriveSpeed,
                turnSpeed * tempDriveSpeed
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
