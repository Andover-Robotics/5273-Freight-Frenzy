package org.firstinspires.ftc.teamcode.a_opmodes.teleop.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.a_opmodes.teleop.BaseOpMode;
import org.firstinspires.ftc.teamcode.b_hardware.subsystems.Hubs;


@TeleOp(name = "IntakeTuningGraphing", group = "ZDashBoard graphing tool")
public class IntakeGraphingOpMode extends BaseOpMode {

    TelemetryPacket packet = new TelemetryPacket();

    private Hubs hubs;

    public void subInit() {
        hubs = new Hubs(this);

        bot.intake.runRight();
        bot.intake.runLeft();

        packet.addLine("Init done");
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    public void subLoop() {
        packet.put("Left Intake Amperage", hubs.leftIntakeCurrentDraw());
        packet.put("Right Intake Amperage", hubs.rightIntakeCurrentDraw());

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }


}
