package org.firstinspires.ftc.teamcode.a_opmodes.teleop.tuning;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.b_hardware.subsystems.Hubs;
import org.firstinspires.ftc.teamcode.d_util.utilclasses.TimingScheduler;

@TeleOp(group = "Competition", name = "Amperage Calibration")
public class AmperageCalibrationOpMode extends OpMode {

    private TimingScheduler timingScheduler;
    private MotorEx rightIntakeMotor;
    private MotorEx leftIntakeMotor;
    private Hubs hubs;
    private double time = this.getRuntime();
    private double leftIntakeAmperage, rightIntakeAmperage;

    @Override
    public void init() {
        timingScheduler = new TimingScheduler(this);
        
        hubs = new Hubs(this);
    }

    @Override
    public void loop() {

        telemetry.addData("Cycle Time", this.getRuntime() - time);
        telemetry.addData("Left Amperage", hubs.leftIntakeCurrentDraw());
        telemetry.addData("Right Amperage", hubs.rightIntakeCurrentDraw());

        leftIntakeAmperage = Math.max(hubs.leftIntakeCurrentDraw(), leftIntakeAmperage);
        rightIntakeAmperage = Math.max(hubs.rightIntakeCurrentDraw(), rightIntakeAmperage);

        GlobalConfig.leftIntakeThreshold = leftIntakeAmperage;
        GlobalConfig.rightIntakeThreshold = rightIntakeAmperage;

        timingScheduler.run();

    }
}
