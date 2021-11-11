package org.firstinspires.ftc.teamcode.b_hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.GlobalConfig;

@Config
public class Carousel extends SubsystemBase {
    public static final double MAX_SPEED = (int) 1150 / 60 * 145.1;
    public static final double SPEED_PERCENT = 1150 / 750;
    public double targetSpeed = 0.0;
    private MotorEx motor;

    private enum state {
        ON,
        OFF
    }
    private state runState = state.OFF;


    public Carousel(OpMode opMode){
        motor = new MotorEx(opMode.hardwareMap, "carousel", Motor.GoBILDA.RPM_1150);
        motor.setRunMode(Motor.RunMode.VelocityControl);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor.motor.setDirection(GlobalConfig.alliance == GlobalConfig.Alliance.RED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    public void runToggle() {
        if(runState == state.OFF) {
            run();
        }
        else if(runState == state.ON) {
            stop();
        }
    }

    public void run() {
        motor.motorEx.setVelocity(MAX_SPEED * SPEED_PERCENT);
        runState = state.ON;
    }

    public void stop() {
        motor.motorEx.setVelocity(0.0);
        runState = state.OFF;
    }




}
