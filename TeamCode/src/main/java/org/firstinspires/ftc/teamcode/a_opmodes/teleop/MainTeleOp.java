package org.firstinspires.ftc.teamcode.a_opmodes.teleop;

import android.hardware.TriggerEvent;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.util.Direction;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.b_hardware.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.c_drive.RRMecanumDrive.Mode;
import org.firstinspires.ftc.teamcode.d_util.utilclasses.TimingScheduler;

import java.util.Map;
import java.util.Map.Entry;

@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends BaseOpMode {//required vars here
  private double cycle = 0;
  private double prevRead = 0;
  private TimingScheduler timingScheduler;
  private boolean centricity = false;
  private boolean isManual = true;
  private int percent = 1, part = 0;
  private double triggerConstant = 0.05;
  double slowModeSpeed = 0.4;




  //config? stuff here =========================================================================

  private double fieldCentricOffset = -90.0;
  public enum TemplateState{
    INTAKE(0.5),
    TRANSPORT(0.5),
    OUTTAKE(0.5);

    public final double progressRate;

    TemplateState(double progressRate){this.progressRate = progressRate;}
  }
// test edit
  Map<TemplateState, Map<Button, TemplateState>> stateMap = new StateMap().getStateMap();

  public TemplateState state = TemplateState.INTAKE;


  //opmode vars here ==============================================================================================
  //If there is a module-specific var, put it in the module class ie slideStage goes in the slides module


//  private MotorEx leftIntake;
//  private MotorEx rightIntake;

  public void subInit() {
    //TODO: initialize subsystems not initialized in bot constructor
    timingScheduler = new TimingScheduler(this);

  }

  @Override
  public void subLoop() {
    //update stuff=================================================================================================
    cycle = 1.0/(time-prevRead);
    prevRead = time;
    timingScheduler.run();

    //Movement =================================================================================================
    //TODO: change depending on mode
    driveSpeed = 1;

    if(justPressed(Button.BACK)){
      isManual = !isManual;
    }

    if (isManual) {
      drive();
    }

    else {
      followPath();
    }

    // intake controls
    if (gamepadEx1.isDown(Button.LEFT_BUMPER)){
      bot.intake.reverseLeft();
    }

    else if (gamepadEx1.isDown(Button.RIGHT_BUMPER)) {
      bot.intake.reverseRight();
    }

    else if (gamepadEx1.getTrigger(Trigger.RIGHT_TRIGGER) > triggerConstant) {
      bot.intake.runRight();
    }

    else if (gamepadEx1.getTrigger(Trigger.LEFT_TRIGGER) > triggerConstant){
      bot.intake.runLeft();
    }

    else if (gamepadEx1.getButton(Button.B)){
      bot.intake.elementIntook = false;
    }

    else {
      bot.intake.stop();
    }

    if (bot.intake.elementIntook) {
      if (bot.intake.reverseLeft) {
        bot.outtake.closeRightFlap();
      }
      else {
        bot.outtake.closeLeftFlap();
      }
    }

    // driver 2

    // toggling flaps to hold freight in bucket
    if (gamepadEx2.wasJustReleased(Button.LEFT_BUMPER)){
      bot.outtake.toggleLeftFlap();
    }

    else if (gamepadEx2.wasJustReleased(Button.RIGHT_BUMPER)){
      bot.outtake.toggleRightFlap();
    }

    // all slides controls
    if(gamepadEx2.wasJustReleased(Button.LEFT_STICK_BUTTON)) {
      bot.outtake.goToCapstone();
    }

    else if(gamepadEx2.wasJustPressed(Button.RIGHT_STICK_BUTTON)) {
      bot.outtake.hookCapstone();
    }

    else if(gamepadEx2.wasJustReleased(Button.DPAD_DOWN)) {
      bot.outtake.goToLowGoal();
    }

    else if(gamepadEx2.wasJustReleased(Button.DPAD_UP)) {
      bot.outtake.goToTopGoal();
    }

    else if(gamepadEx2.wasJustReleased(Button.DPAD_RIGHT)) {
      bot.outtake.fullyRetract();

    }

    else if (gamepadEx2.wasJustReleased(Button.Y)){
      bot.outtake.toggleBucket();
    }

    // carousel controls
    if (gamepadEx2.wasJustReleased(Button.A)){
      bot.carousel.toggleBlue();
    }

    else if (gamepadEx2.wasJustReleased(Button.B)) {
      bot.carousel.toggleRed();
    }

    else if (gamepadEx2.wasJustReleased(Button.X)) {
      bot.carousel.stop();
    }





    /*//TODO: make control scheme
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

    


    /*
    AUTOMATION CONTROL SCHEME

     */



    CommandScheduler.getInstance().run();

    // TODO organize this test code
    updateLocalization();
    telemetry.addData("Right Detected?", bot.intake.intookRight);
    telemetry.addData("Left Detected?", bot.intake.intookLeft);
    telemetry.addData("Right Amperage:", bot.intake.rightIntakeCurrentDraw());
    telemetry.addData("Left Amperage:", bot.intake.leftIntakeCurrentDraw());
    telemetry.addData("Right Intake", bot.intake.isRightIntaking());
    telemetry.addData("Left Intake", bot.intake.isLeftIntaking());
    telemetry.addData("Right Intake", bot.intake.runState);
    /*
    telemetry.addData("percent", percent);
    telemetry.addData("part", part);
    telemetry.addData("cycle", cycle);
    telemetry.addData("x", bot.roadRunner.getPoseEstimate().getX());
    telemetry.addData("y", bot.roadRunner.getPoseEstimate().getY());
    telemetry.addData("heading", bot.roadRunner.getPoseEstimate().getHeading());
    telemetry.addData("current raw angle", bot.imu.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).firstAngle);

     */
  }


  private void drive(){//Driving ===================================================================================
    final double gyroAngle =
            bot.imu.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).secondAngle//TODO: make sure that the orientation is correct
                    - fieldCentricOffset;
    Vector2d driveVector = stickSignal(Direction.LEFT),
            turnVector = new Vector2d(
                    stickSignal(Direction.RIGHT).getX() * Math.abs(stickSignal(Direction.RIGHT).getX()),
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
                driveVector.getX() * driveSpeed,
                turnVector.getX() * driveSpeed,
                gyroAngle);

      else if (dpadPressed || buttonPressed)
        bot.drive.driveRobotCentric(
                - strafeSpeed * slowModeSpeed,
                - forwardSpeed * slowModeSpeed,
                turnSpeed * slowModeSpeed
          );

      else
        bot.drive.driveRobotCentric(
                driveVector.getY() * driveSpeed,
                driveVector.getX() * -driveSpeed,
                turnVector.getX() * driveSpeed
        );

    }
    if (justPressed(Button.LEFT_STICK_BUTTON)) {
      fieldCentricOffset = bot.imu.getAngularOrientation()
          .toAngleUnit(AngleUnit.DEGREES).firstAngle;
    }
    if (justPressed(Button.RIGHT_STICK_BUTTON)){
      centricity = !centricity;
    }

  }

  private void followPath(){//Path following ===================================================================================

    updateState();

  }

  private void updateState(){
    for(Entry<Button, TemplateState> pair : stateMap.get(state).entrySet()){
      if(justPressed(pair.getKey())){
        state = pair.getValue();
        percent = 0;
      }
    }
  }

  private void updateLocalization() {
    bot.roadRunner.update();
  }
}
