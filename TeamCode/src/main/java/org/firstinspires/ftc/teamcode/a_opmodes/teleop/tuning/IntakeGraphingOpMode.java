package org.firstinspires.ftc.teamcode.a_opmodes.teleop.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.a_opmodes.teleop.BaseOpMode;
import org.firstinspires.ftc.teamcode.b_hardware.subsystems.Hubs;


@TeleOp(name = "IntakeTuningGraphing", group = "ZDashBoard graphing tool")
public class IntakeGraphingOpMode extends BaseOpMode {

    TelemetryPacket packet = new TelemetryPacket();

    private static final double TRIGGER_TOLERANCE = 0.05;

    private static double lastRightAmperage, lastLeftAmperage;

    private MotorEx leftIntake, rightIntake;

    private Hubs hubs;

    public void subInit() {
        hubs = new Hubs(this);
        packet.addLine("Init done");
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    public void subLoop() {

        lastLeftAmperage = hubs.leftIntakeCurrentDraw();
        lastRightAmperage = hubs.rightIntakeCurrentDraw();

        packet.put("Left Intake Amperage", lastLeftAmperage);
        packet.put("Right Intake Amperage", lastRightAmperage);


        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        /*

        double leftIntakeVelo = bot.intake.leftIntake.getVelocity(),
               rightIntakeVelo = bot.intake.rightIntake.getVelocity();

        if(gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= TRIGGER_TOLERANCE) {
            bot.intake.runRight();
        }
        else {
            bot.intake.stop();
        }
        if(gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >= TRIGGER_TOLERANCE) {
            bot.intake.runLeft();
        }
        else {
            bot.intake.stop();
        }

        packet.put("Left Intake Accel", leftIntakeVelo - lastLeftRPM);
        packet.put("Right Intake Accel", lastRightRPM - rightIntakeVelo);

        lastLeftRPM = leftIntakeVelo;
        lastRightRPM = rightIntakeVelo;

        packet.put("Left Intake RPM", leftIntakeVelo);
        packet.put("Right Intake RPM", rightIntakeVelo);


        FtcDashboard.getInstance().sendTelemetryPacket(packet);

         */
    }


}