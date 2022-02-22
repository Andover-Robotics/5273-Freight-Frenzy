package org.firstinspires.ftc.teamcode.a_opmodes.teleop.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.a_opmodes.teleop.BaseOpMode;

@TeleOp(name = "IntakeDetectionTesting", group = "ZTesting")
public class IntakeDetectionTestingOpMode extends BaseOpMode {


    TelemetryPacket packet = new TelemetryPacket();
    double prevLeftVelo, prevRightVelo;

    private static final double TRIGGER_TOLERANCE = 0.1;
    private static final double INTAKE_DETECT_CONST = 1150;

    @Override
    public void subInit() {
        packet.addLine("Init done");
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }


    @Override
    public void subLoop() {

        double leftIntakeVelo = bot.intake.leftIntake.getVelocity(),
                rightIntakeVelo = bot.intake.rightIntake.getVelocity();

        if(gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= TRIGGER_TOLERANCE) {
            bot.intake.runRight();
        }
        else if(gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >= TRIGGER_TOLERANCE) {
            bot.intake.runLeft();
        }
        else {
            bot.intake.stop();
        }

        if(leftIntakeVelo < INTAKE_DETECT_CONST && !bot.intake.isLeftIntaking()) packet.put("Left Intaked", 300);
        else packet.put("Left Intaked", 0);
        if(rightIntakeVelo < INTAKE_DETECT_CONST && !bot.intake.isRightIntaking()) packet.put("Right Intaked", 300);
        else packet.put("Right Intaked", 0);

        packet.put("Left Intake Accel", leftIntakeVelo - prevLeftVelo);
        packet.put("Right Intake Accel", prevRightVelo - rightIntakeVelo);

        prevLeftVelo = leftIntakeVelo;
        prevRightVelo = rightIntakeVelo;

        packet.put("Left Intake RPM", leftIntakeVelo);
        packet.put("Right Intake RPM", rightIntakeVelo);


        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
