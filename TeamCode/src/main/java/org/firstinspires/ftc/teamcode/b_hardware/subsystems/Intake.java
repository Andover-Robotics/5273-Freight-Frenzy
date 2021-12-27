package org.firstinspires.ftc.teamcode.b_hardware.subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


public class Intake extends SubsystemBase {
    public static final double SPEED = 1.0;
    public MotorEx leftIntake;
    public MotorEx rightIntake;

    private double prevLeftVelo, curLeftVelo;
    private double prevRightVelo, curRightVelo;
    private final double INTAKE_DETECT_CONST = 100;

    private enum state {
        ON,
        OFF,
        REVERSE,
        LEFT,
        RIGHT
    }

    public state runState = state.OFF;

    public Intake(@NonNull OpMode opMode){
        // TODO: make a mojor change and run the intake to a specific velocity, so cube weight detection will work after calibration
        //      for example, calibrate to a set velocity, and run the intakes at that set velocity during the duration of the match
        leftIntake = new MotorEx(opMode.hardwareMap, "leftIntake", Motor.GoBILDA.RPM_312);
        leftIntake.setRunMode(Motor.RunMode.RawPower);
        leftIntake.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntake = new MotorEx(opMode.hardwareMap, "rightIntake", Motor.GoBILDA.RPM_312);
        rightIntake.setRunMode(Motor.RunMode.RawPower);
        rightIntake.motor.setDirection(DcMotorSimple.Direction.FORWARD);
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

        curLeftVelo = leftIntake.getCorrectedVelocity();
        curRightVelo = rightIntake.getCorrectedVelocity();
    }

    public boolean wasIntakedLeft() {
        return Math.abs(curLeftVelo/prevLeftVelo) > INTAKE_DETECT_CONST;
    }

    public boolean wasIntakedRight() {
        return Math.abs(curRightVelo/prevRightVelo) > INTAKE_DETECT_CONST;
    }

}
