package org.firstinspires.ftc.teamcode.a_opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.util.Direction;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.b_hardware.Bot;
import org.firstinspires.ftc.teamcode.d_util.utilclasses.TimingScheduler;

import java.util.function.Function;


public abstract class BaseOpMode extends OpMode {

  protected Bot bot;
  protected double driveSpeed;
  protected GamepadEx gamepadEx1, gamepadEx2;
  TimingScheduler timingScheduler;

  //button reader syntax
  // (g1 or g2)  (a, b, lt, lb, etc)

  @Override
  public void init() {
    bot = Bot.getInstance(this);
    bot.init();
    gamepadEx2 = new GamepadEx(gamepad2);
    gamepadEx1 = new GamepadEx(gamepad1);
    timingScheduler = new TimingScheduler(this);
    subInit();
    telemetry.addLine("Init done");
    telemetry.update();
  }

  @Override
  public void loop() {
    updateButtons();
    CommandScheduler.getInstance().run();
    subLoop();
  }

  protected abstract void subInit();

  protected abstract void subLoop();

  void updateButtons(){
    gamepadEx1.readButtons();
    gamepadEx2.readButtons();
  }



  protected boolean buttonSignal(GamepadEx gp, Button button) {
    return gp.isDown(button);
  }

  protected double triggerSignal(GamepadEx gp, Trigger trigger) {
    return gp.getTrigger(trigger);
  }

  protected Vector2d stickSignal(Direction side) {
    Function<GamepadEx, Vector2d> toCoords = pad ->
        side == Direction.LEFT ? new Vector2d(pad.getLeftX(), pad.getLeftY()) :
            new Vector2d(pad.getRightX(), pad.getRightY());

    Vector2d v1 = toCoords.apply(gamepadEx1),
        v2 = toCoords.apply(gamepadEx2);

    return v1.magnitude() > 0.02 ? v1 : v2;
  }

  protected boolean justPressed(GamepadEx gp, Button button) {
    return gp.wasJustPressed(button);
  }

  protected boolean justReleased(GamepadEx gp, Button button){
    return !(gp.isDown(button)) && (gp.wasJustReleased(button));
  }

}
