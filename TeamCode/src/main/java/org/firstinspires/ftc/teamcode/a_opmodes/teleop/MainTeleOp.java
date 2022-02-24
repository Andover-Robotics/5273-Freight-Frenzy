package org.firstinspires.ftc.teamcode.a_opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.b_hardware.Bot;
import org.firstinspires.ftc.teamcode.c_drive.RRMecanumDrive;
import org.firstinspires.ftc.teamcode.d_util.utilclasses.TimingScheduler;

@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends BaseOpMode {//required vars here
  private double cycle = 0;
  private double prevRead = 0;
  private boolean centricity = false;
  private final double TRIGGER_CONSTANT = 0.15;
  private final double SLOW_MODE_PERCENT = 0.4;
  private double fieldCentricOffset0 = 0.0;
  private double fieldCentricOffset1 = 0.0;

  private double carouselStartTime = 0.0;



  //config? stuff here =========================================================================

  //opmode vars here ==============================================================================================
  public void subInit() {
    driveSpeed = 1.0;
    bot.outtake.closeRightFlap();
    bot.outtake.closeLeftFlap();
    bot.outtake.unFlipBucket();
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
      bot.outtake.closeRightFlap();
      bot.outtake.openLeftFlap();
    }
    else {
      bot.intake.stopLeft();
    }

    if (gamepadEx1.isDown(Button.RIGHT_BUMPER)) {
      bot.intake.reverseRight();
    }
    else if (gamepadEx1.getTrigger(Trigger.RIGHT_TRIGGER) > TRIGGER_CONSTANT) {
      bot.intake.runRight();
      bot.outtake.openRightFlap();
      bot.outtake.closeLeftFlap();
    }
    else {
      bot.intake.stopRight();
    }

    if(gamepadEx1.wasJustPressed(Button.DPAD_RIGHT) || gamepadEx2.wasJustPressed(Button.RIGHT_STICK_BUTTON)) {
      bot.outtake.setAutoFlap(true);
    }

    if(Math.abs(gamepadEx2.getLeftY()) > TRIGGER_CONSTANT) {
      bot.outtake.addOffset(gamepadEx2.getLeftY());
    }

    if (gamepadEx2.wasJustPressed(Button.LEFT_BUMPER)) {
      bot.outtake.setAutoFlap(false);
      bot.outtake.openLeftFlap();

    }
    else if(gamepadEx2.getTrigger(Trigger.LEFT_TRIGGER) > TRIGGER_CONSTANT) {
      bot.outtake.closeLeftFlap();
    }

    if (gamepadEx2.wasJustPressed(Button.RIGHT_BUMPER)) {
      bot.outtake.setAutoFlap(false);
      bot.outtake.openRightFlap();
    }
    else if(gamepadEx2.getTrigger(Trigger.RIGHT_TRIGGER) > TRIGGER_CONSTANT) {
      bot.outtake.closeRightFlap();
    }

    // all slides controls
    if(gamepadEx2.wasJustPressed(Button.LEFT_STICK_BUTTON)) {
      bot.outtake.goToCapstone();
    }
    else if(gamepadEx2.wasJustPressed(Button.RIGHT_STICK_BUTTON)){
      bot.outtake.hookCapstone();
      timingScheduler.defer(1.0, () -> {
        bot.outtake.hookTSE();
      });
      timingScheduler.defer(2.0, () -> {
        bot.outtake.capstone();
      });
    }
    else if (Math.abs(gamepadEx2.getRightY()) > TRIGGER_CONSTANT) {
      bot.outtake.incrementBucket(bot.outtake.bucket.getPosition() + gamepadEx2.getRightY() * 0.07);
    }
    else if(gamepadEx2.wasJustPressed(Button.DPAD_UP)) {
      bot.outtake.goToTopGoal();
    }
    else if(gamepadEx2.wasJustPressed(Button.DPAD_LEFT)) {
      bot.outtake.goToLowGoal();
    }
    else if(gamepadEx2.wasJustPressed(Button.DPAD_RIGHT)) {
      bot.outtake.goToMidGoal();
    }
    else if(gamepadEx2.wasJustPressed(Button.DPAD_DOWN)) {
      bot.outtake.fullyRetract();
    }

    if(gamepadEx2.isDown(Button.X)) {
      bot.outtake.flipBucket();
    }
    else if(gamepadEx2.wasJustReleased(Button.X)) {
      bot.outtake.fullyRetract();
    }


    // carousel controls

    if(gamepadEx2.wasJustPressed(Button.B) && Math.abs(carouselStartTime - this.time) > 1.5) {
      carouselStartTime = this.time;
    }
    else if(gamepadEx2.wasJustPressed(Button.Y)){
      carouselStartTime -= 1.2;
    }

    runCarousel();



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

    telemetry.addData("Centricity", centricity);
    telemetry.addData("cycle", cycle);
    telemetry.addData("x", bot.roadRunner.getPoseEstimate().getX());
    telemetry.addData("y", bot.roadRunner.getPoseEstimate().getY());
    telemetry.addData("heading", bot.roadRunner.getPoseEstimate().getHeading());
    telemetry.addData("driver left stick", "left X" + gamepadEx1.getLeftX() + ": " + gamepadEx1.getLeftY());
    telemetry.addLine("isFreightIn : " + bot.outtake.isFreightIn());
    telemetry.addLine("autoFlap: " + bot.outtake.isAutoFlap());
    telemetry.addLine("System time: " + this.time);
    telemetry.addLine("startCarouselTime: " + carouselStartTime);
    telemetry.addData("Carousel Speed", bot.carousel.getMotor().getVelocity());
  }

  private void runCarousel() {
    double curTime = Math.abs(carouselStartTime - this.time);
    if(curTime < 0.7) {
      bot.carousel.runAtTPS(GlobalConfig.alliance == GlobalConfig.Alliance.RED,
              (Math.pow(1.5, curTime * 0.3) - 0.8) * bot.carousel.getMotor().ACHIEVABLE_MAX_TICKS_PER_SECOND);
    }
    else if (curTime < 1.2) {
      bot.carousel.runAtTPS(GlobalConfig.alliance == GlobalConfig.Alliance.RED,
              (Math.pow(4, curTime * 0.25) - 0.99) * bot.carousel.getMotor().ACHIEVABLE_MAX_TICKS_PER_SECOND);
    }
    else {
      bot.carousel.stop();
    }
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

      if (centricity) {//epic java syntax
        bot.drive.driveFieldCentric(
                driveVector.getY() * driveSpeed,
                driveVector.getX() * -driveSpeed,
                turnVector.getX() * driveSpeed,
                (Math.abs(avgGyroAngle - gyroAngle0) < gyroTolerance
                        || Math.abs(avgGyroAngle - gyroAngle1) < gyroTolerance) ?
                        Math.abs(gyroAngle0 - avgGyroAngle) <
                                Math.abs(gyroAngle1 - avgGyroAngle) ?
                                gyroAngle0 : gyroAngle1 : avgGyroAngle
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

    if (gamepadEx1.wasJustPressed(Button.LEFT_STICK_BUTTON)) {
      fieldCentricOffset0 = bot.imu0.getAngularOrientation()
              .toAngleUnit(AngleUnit.DEGREES).firstAngle;
      fieldCentricOffset1 = bot.imu1.getAngularOrientation()
              .toAngleUnit(AngleUnit.DEGREES).firstAngle;
    }
    if (gamepadEx1.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)){
      centricity = !centricity;
      if(centricity) {
        fieldCentricOffset0 = bot.imu0.getAngularOrientation()
                .toAngleUnit(AngleUnit.DEGREES).firstAngle;
        fieldCentricOffset1 = bot.imu1.getAngularOrientation()
                .toAngleUnit(AngleUnit.DEGREES).firstAngle;
      }
    }

  }
}
