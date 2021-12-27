package org.firstinspires.ftc.teamcode.b_hardware.subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


public class Intake extends SubsystemBase {


    public final Command cmdRunLeft = new InstantCommand(this::runLeft, this);
    public final Command cmdRunRight = new InstantCommand(this::runRight, this);
    public final Command cmdStopIntake = new InstantCommand(this::stop, this);

    public static final double SPEED = .6;
    public MotorEx leftIntake;
    public MotorEx rightIntake;

    private double prevLeftVelo, curLeftVelo;
    private double prevRightVelo, curRightVelo;
    private final double INTAKE_DETECT_CONST = 100;
    private final double IS_RUNNING_CONST = 15;

    private enum state {
        ON,
        OFF,
        REVERSE,
        LEFT,
        RIGHT
    }

    public state runState = state.OFF;

    public Intake(@NonNull OpMode opMode){
        leftIntake = new MotorEx(opMode.hardwareMap, "leftIntake", Motor.GoBILDA.RPM_312);
        leftIntake.setRunMode(Motor.RunMode.RawPower);
        leftIntake.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntake = new MotorEx(opMode.hardwareMap, "rightIntake", Motor.GoBILDA.RPM_312);
        rightIntake.setRunMode(Motor.RunMode.RawPower);
        rightIntake.motor.setDirection(DcMotorSimple.Direction.FORWARD);
        prevLeftVelo = 0;
        prevRightVelo = 0;
        curRightVelo = 0;
        curLeftVelo = 0;


    }

    @Override
    public void periodic() {
        updateVeloVals();
    }

    public void runToggle() {
        if(runState == state.OFF) {
            run();
        }
        else {
            stop();
        }
    }

    public void run(){
        leftIntake.set(SPEED);
        rightIntake.set(SPEED);
        runState = state.ON;
    }

//    public void reverse() {
//        runState = state.REVERSE;
//        leftIntake.set(-SPEED);
//        rightIntake.set(-SPEED);
//    }
//    deprecated

    public void reverseLeft() {
        leftIntake.set(-SPEED);
    }

    public void reverseRight() {
        rightIntake.set(-SPEED);
    }

    public void runLeft(){
        rightIntake.set(0.0);
        leftIntake.set(SPEED);
        runState = state.LEFT;
    }

    public void runRight(){
        leftIntake.set(0.0);
        rightIntake.set(SPEED);
        runState = state.RIGHT;
    }

    public void switchIntake(){
        if (runState == state.LEFT) {
            runRight();
        }
        else {
            runLeft();
        }
    }

    public void stop(){
        leftIntake.set(0.0);
        rightIntake.set(0.0);
        runState = state.OFF;
    }

    private void updateVeloVals() {
        prevRightVelo = curRightVelo;
        prevLeftVelo  = curLeftVelo;

        curLeftVelo = leftIntake.getVelocity();
        curRightVelo = rightIntake.getVelocity();
    }

    public boolean wasIntakedLeft() {
        return Math.abs(curLeftVelo - prevLeftVelo) > INTAKE_DETECT_CONST;
    }

    public boolean wasIntakedRight() {
        return Math.abs(curRightVelo - prevRightVelo) > INTAKE_DETECT_CONST;
    }

    public double getCurLeftVelo() { return curLeftVelo; }
    public double getCurRightVelo() { return curRightVelo; }
    public double getPrevLeftVelo() { return prevLeftVelo; }
    public double getPrevRightVelo() { return prevRightVelo; }

    public boolean isLeftRunning() {
        return leftIntake.getVelocity() < IS_RUNNING_CONST;
    }
    public boolean isRightRunning() {
        return rightIntake.getVelocity() < IS_RUNNING_CONST;
    }

}
