package org.firstinspires.ftc.teamcode.a_opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.c_drive.RRMecanumDrive.Mode;
import org.firstinspires.ftc.teamcode.d_util.utilclasses.TimingScheduler;

@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends BaseOpMode {//required vars here
  private double cycle = 0;
  private double prevRead = 0;
  private TimingScheduler timingScheduler;
  private boolean centricity = false;
  private final double TRIGGER_CONSTANT = 0.15;
  private final double SLOW_MODE_SPEED = 0.4;

  //config? stuff here =========================================================================

  private double fieldCentricOffset = 0.0;

  //opmode vars here ==============================================================================================
  public void subInit() {
    timingScheduler = new TimingScheduler(this);
  }

  @Override
  public void subLoop() {
    //update stuff=================================================================================================
    cycle = 1.0/(time-prevRead);
    prevRead = time;
    timingScheduler.run();

    //Movement =================================================================================================
    drive();


    //Subsystem control =========================================================================================

    // intake controls
    if (gamepadEx1.isDown(Button.LEFT_BUMPER)){
      bot.intake.reverseLeft();
    }
    else if (gamepadEx1.getTrigger(Trigger.LEFT_TRIGGER) > TRIGGER_CONSTANT){
      bot.intake.runLeft();
    }
    else {
      bot.intake.stopLeft();
    }

    if (gamepadEx1.isDown(Button.RIGHT_BUMPER)) {
      bot.intake.reverseRight();
    }
    else if (gamepadEx1.getTrigger(Trigger.RIGHT_TRIGGER) > TRIGGER_CONSTANT) {
      bot.intake.runRight();
    }
    else {
      bot.intake.stopRight();
    }


      // driver 2

    // toggling flaps to hold freight in bucket
    if (gamepadEx2.wasJustPressed(Button.LEFT_BUMPER)){
      bot.outtake.toggleLeftFlap();
    }

    else if (gamepadEx2.wasJustPressed(Button.RIGHT_BUMPER)){
      bot.outtake.toggleRightFlap();
    }

    // all slides controls
    if(gamepadEx2.wasJustPressed(Button.LEFT_STICK_BUTTON)) {
      bot.outtake.goToCapstone();
    }
    else if(gamepadEx2.wasJustPressed(Button.DPAD_DOWN)) {
      bot.outtake.goToLowGoal();
    }
    else if(gamepadEx2.wasJustPressed(Button.DPAD_LEFT)) {
      bot.outtake.goToMidGoal();
    }
    else if(gamepadEx2.wasJustPressed(Button.DPAD_UP)) {
      bot.outtake.goToTopGoal();
    }
    else if(gamepadEx2.wasJustPressed(Button.DPAD_RIGHT)) {
      bot.outtake.fullyRetract();

    }
    else if (gamepadEx2.wasJustPressed(Button.Y)){
      bot.outtake.toggleBucket();
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


    /*
    Controller 1
    A:      B:      X:      Y:
    DPAD
    L: Unflip BucketD:     U: Flip Bucket R:
    Joystick
    L:Field centric movement
    R:Set orientation / Rotation (Determine through practice)
    Trigger L/R: left intake -- right intake
    Bumper:
    L:none/switch to previous path      R:none/switch to next path
    Other
    Start:  Back:switch between automation and driving

    Controller 2
    A:      B:      X:      Y:
    DPAD
    L:      D: Unflip Bucket    U: Flip Bucket     R:
    Joystick
    L:movement/reset field centric or progress automation
    R:movement/switch robotfield centric or none
    Trigger L/R: slow driving
    Bumper
    L: Open Left Flap, Close Right Flap      R: Open Right Flap, Close Left Flap
    Other
    Start:  Back:switch between automation and driving
     */

    CommandScheduler.getInstance().run();

    telemetry.addData("Centricity", centricity);
    telemetry.addData("cycle", cycle);
    telemetry.addData("x", bot.roadRunner.getPoseEstimate().getX());
    telemetry.addData("y", bot.roadRunner.getPoseEstimate().getY());
    telemetry.addData("heading", bot.roadRunner.getPoseEstimate().getHeading());
    telemetry.addData("current raw angle", bot.imu.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).firstAngle);
  }


  private void drive(){//Driving ===================================================================================
    final double gyroAngle =
            bot.imu.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).firstAngle
                    - fieldCentricOffset;
    Vector2d driveVector = new Vector2d(gamepadEx1.getLeftX(), gamepadEx1.getLeftY()),
            turnVector = new Vector2d(
                    gamepadEx1.getRightX() * Math.abs(gamepadEx1.getRightX()),
                    0);
    if (bot.roadRunner.mode == Mode.IDLE) {

      boolean dpadPressed = (gamepadEx1.getButton(Button.DPAD_DOWN) || gamepadEx1.getButton(Button.DPAD_UP)
              || gamepadEx1.getButton(Button.DPAD_LEFT) || gamepadEx1.getButton(Button.DPAD_RIGHT));
      boolean buttonPressed = (gamepadEx1.getButton(Button.X) || gamepadEx1.getButton(Button.B));
      double forwardSpeed = (gamepadEx1.getButton(Button.DPAD_LEFT) || gamepadEx1.getButton(Button.DPAD_RIGHT)) ? (gamepadEx1.getButton(Button.DPAD_RIGHT) ? 1 : -1) : 0;
      double strafeSpeed = (gamepadEx1.getButton(Button.DPAD_DOWN) || gamepadEx1.getButton(Button.DPAD_UP)) ? (gamepadEx1.getButton(Button.DPAD_UP) ? 1 : -1) : 0;
      double turnSpeed = (gamepadEx1.getButton(Button.X) || gamepadEx1.getButton(Button.B)) ? (gamepadEx1.getButton(Button.B) ? 1 : -1) : 0;

      if (centricity) //epic java syntax
        bot.drive.driveFieldCentric(
                driveVector.getY() * driveSpeed,
                driveVector.getX() * -driveSpeed,
                turnVector.getX() * driveSpeed,
                gyroAngle);

      else if (dpadPressed || buttonPressed)
        bot.drive.driveRobotCentric(
                strafeSpeed * SLOW_MODE_SPEED,
                forwardSpeed * -SLOW_MODE_SPEED,
                turnSpeed * SLOW_MODE_SPEED
          );

      else
        bot.drive.driveRobotCentric(
                driveVector.getY() * driveSpeed,
                driveVector.getX() * -driveSpeed,
                turnVector.getX() * driveSpeed
        );

    }
    if (gamepadEx1.wasJustPressed(Button.LEFT_STICK_BUTTON)) {
      fieldCentricOffset = bot.imu.getAngularOrientation()
          .toAngleUnit(AngleUnit.DEGREES).firstAngle;
    }
    if (gamepadEx1.wasJustPressed(Button.RIGHT_STICK_BUTTON)){
      centricity = !centricity;
    }

  }
}
