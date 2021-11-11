package org.firstinspires.ftc.teamcode.b_hardware.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.R;


public class Intake extends SubsystemBase {
    public static final double SPEED = 0.8;
    public MotorEx leftIntake;
    public MotorEx rightIntake;

    private enum state {
        ON,
        OFF,
        REVERSE,
        LEFT,
        RIGHT
    }

    public state runState = state.OFF;

    public Intake(OpMode opMode){
        leftIntake = new MotorEx(opMode.hardwareMap, "leftIntake", Motor.GoBILDA.RPM_435);
        leftIntake.setRunMode(Motor.RunMode.RawPower);
        rightIntake = new MotorEx(opMode.hardwareMap, "rightIntake", Motor.GoBILDA.RPM_435);
        rightIntake.setRunMode(Motor.RunMode.RawPower);
        rightIntake.motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void runToggle() {
        if(runState == state.OFF) {
            run();
        }
        else if(runState == state.ON) {
            stop();
        }
    }

    public void reverse() {
        runState = state.REVERSE;
        runReverse();
    }

    public void run(){
        leftIntake.set(SPEED);
        rightIntake.set(SPEED);
        runState = state.ON;
    }

    public void runReverse() {
        leftIntake.set(-SPEED);
        rightIntake.set(-SPEED);
    }

    public void switchIntake(){
        if (runState == state.LEFT) {
            leftIntake.set(0.0);
            rightIntake.set(SPEED);
            runState = state.LEFT;
        }
        else {
            rightIntake.set(0.0);
            leftIntake.set(SPEED);
            runState = state.RIGHT;
        }
    }

    public void stop(){
        leftIntake.set(0.0);
        rightIntake.set(0.0);
        runState = state.OFF;
    }
}
