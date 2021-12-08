package org.firstinspires.ftc.teamcode.b_hardware.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.GlobalConfig;

@Config
public class Carousel extends SubsystemBase {
    public static double OPTIMAL_RPM = 531;
    public static final double MOTOR_SPEED = 1150;
    public static final double SPEED_PERCENT = OPTIMAL_RPM / MOTOR_SPEED; //531 rpm


    private final MotorEx motor;

    private enum State {
        RED_ON,
        BLUE_ON,
        OFF
    }
    private State runState = State.OFF;


    public Carousel(@NonNull OpMode opMode){
        motor = new MotorEx(opMode.hardwareMap, "carousel", Motor.GoBILDA.RPM_1150);
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motor.setInverted(GlobalConfig.alliance == GlobalConfig.Alliance.RED);
    }





    public void runRed() {
        motor.set(SPEED_PERCENT);
        runState = State.RED_ON;
    }

    public void runBlue() {
        motor.set(SPEED_PERCENT);
        runState = State.BLUE_ON;
    }

    public void toggleBlue() {
        if(runState == State.BLUE_ON) {
            stop();
        }
        else if(runState == State.RED_ON) {
            stop();
            runBlue();
        }
        else if(runState == State.OFF) {
            runBlue();
        }
    }
    public void toggleRed() {
        if(runState == State.RED_ON) {
            stop();
        }
        else if(runState == State.BLUE_ON) {
            stop();
            runRed();
        }
        else if(runState == State.OFF) {
            runRed();
        }
    }

    public void stop() {
        motor.stopMotor();
        runState = State.OFF;
    }

    public void runAtRPM(double rpmSpeed) {
        motor.set(rpmSpeed / MOTOR_SPEED);
    }

    public MotorEx getMotor() {
        return motor;
    }


}
