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
    public static final double MAX_SPEED = (1150.0 / 60.0 * 145.1);
    public static double OPTIMAL_RPM = 531;
    public static final double SPEED_PERCENT = OPTIMAL_RPM / 1150; //531 rpm
    public static final double RUN_SPEED = MAX_SPEED * SPEED_PERCENT;


    private final MotorEx motor;

    private enum State {
        RED_ON,
        BLUE_ON,
        OFF
    }
    private State runState = State.OFF;


    public Carousel(@NonNull OpMode opMode){
        motor = new MotorEx(opMode.hardwareMap, "carousel", Motor.GoBILDA.RPM_1150);
        motor.setRunMode(Motor.RunMode.VelocityControl);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor.motor.setDirection(GlobalConfig.alliance == GlobalConfig.Alliance.RED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }





    public void runRed() {
        motor.motorEx.setVelocity(RUN_SPEED);
        runState = State.RED_ON;
    }

    public void runBlue() {
        motor.motorEx.setVelocity(-RUN_SPEED);
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
        motor.motorEx.setVelocity(0.0);
        runState = State.OFF;
    }

    public void runAtRPM(double rpmSpeed) {
        double tickSpeed = (1150.0 / 60.0 * 145.1) * (rpmSpeed / 1150);
        motor.motorEx.setVelocity(tickSpeed);
    }




}
