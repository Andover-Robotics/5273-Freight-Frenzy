package org.firstinspires.ftc.teamcode.a_opmodes.auto;

import static org.firstinspires.ftc.teamcode.GlobalConfig.poseEstimate;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.a_opmodes.auto.AutoPaths.AutoPathElement;
import org.firstinspires.ftc.teamcode.a_opmodes.auto.AutoPaths.AutoPathElement.Action;
import org.firstinspires.ftc.teamcode.a_opmodes.auto.AutoPaths.AutoPathElement.Path;
import org.firstinspires.ftc.teamcode.a_opmodes.auto.pipeline.DuckDetector;
import org.firstinspires.ftc.teamcode.a_opmodes.auto.pipeline.DuckDetector.PipelineResult;
import org.firstinspires.ftc.teamcode.b_hardware.Bot;

import java.util.List;

@Autonomous(name = "Main Autonomous", group = "Competition")
public class MainAutonomous extends LinearOpMode {

  private Bot bot;

  PipelineResult detected;
  double confidence;
  DuckDetector pipeline;
  boolean performActions = true;
  GamepadEx gamepad;


  @Override
  public void runOpMode() throws InterruptedException {
    Bot.instance = null;

    bot = Bot.getInstance(this);
    gamepad = new GamepadEx(gamepad1);

    AutoPaths paths = new AutoPaths(this);

    telemetry.addData("Side", GlobalConfig.side);
    telemetry.addData("Alliance", GlobalConfig.alliance);
    telemetry.update();


    pipeline = new DuckDetector(this, telemetry);
    //initialize here
    bot.outtake.fullyRetract();
    bot.outtake.closeRightFlap();
    bot.outtake.closeLeftFlap();

    bot.roadRunner.setPoseEstimate(paths.initialPosition());

    //  ie set servo position                             ========================================================================


    //Pipeline stuff

    while (!isStarted()) {
      if (isStopRequested())
        return;
      // keep getting results from the pipeline
      pipeline.currentlyDetected()
          .ifPresent((pair) -> {
            telemetry.addData("detected", pair.first);
            telemetry.addData("confidence", pair.second);
            telemetry.update();
            detected = pair.first;
            confidence = pair.second;
          });
      if (gamepad1.x) {
        performActions = false;
      }
      if (gamepad1.y) {
        pipeline.saveImage();
      }
    }

    pipeline.currentlyDetected().ifPresent(pair -> {
      detected = pair.first;
      confidence = pair.second;
    });

    if (detected == PipelineResult.NONE)
      detected = PipelineResult.LEFT;

    waitForStart();
    List<AutoPathElement> trajectories = paths.getTrajectories(detected);
    pipeline.close();


    //Roadrunner stuff


    if (isStopRequested())
      return;

    for (AutoPathElement item : trajectories) {

      telemetry.addData("executing path element", item.getName());
      telemetry.update();

      if (item instanceof AutoPathElement.Path) {
        bot.roadRunner.followTrajectory(((Path) item).getTrajectory());
      } else if (item instanceof AutoPathElement.Action && performActions) {
        ((Action) item).getRunner().invoke();
      }

      if (isStopRequested())
        return;
    }

    poseEstimate = bot.roadRunner.getPoseEstimate();
  }
}