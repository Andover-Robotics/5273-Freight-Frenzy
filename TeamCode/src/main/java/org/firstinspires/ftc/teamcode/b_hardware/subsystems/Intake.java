package org.firstinspires.ftc.teamcode.b_hardware.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


public class Intake extends SubsystemBase {
    public static final double SPEED = 1;
    public MotorEx leftIntake;
    public MotorEx rightIntake;

    public enum state {
        ON,
        OFF,
        REVERSE,
        LEFT,
        RIGHT
    }

    public state toggleState = state.OFF;

    public Intake(OpMode opMode){
        leftIntake = new MotorEx(opMode.hardwareMap, "leftIntake", Motor.GoBILDA.RPM_435);
        leftIntake.setRunMode(Motor.RunMode.RawPower);
        rightIntake = new MotorEx(opMode.hardwareMap, "rightIntake", Motor.GoBILDA.RPM_435);
        rightIntake.setRunMode(Motor.RunMode.RawPower);
        rightIntake.motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void toggle() {
        if(toggleState == state.ON) {
            stop();
        }
        else {
            run();
        }
    }

    public void switchIntake(){
        if (toggleState == state.LEFT) {
            leftIntake.set(0.0);
            rightIntake.set(SPEED);
            toggleState = state.LEFT;
        }
        else {
            rightIntake.set(0.0);
            leftIntake.set(SPEED);
            toggleState = state.RIGHT;
        }
    }

    public void run(){
        leftIntake.set(SPEED);
        rightIntake.set(SPEED);
        toggleState = state.ON;
    }

    public void outtake(){
        leftIntake.set(- SPEED);
        rightIntake.set(- SPEED);
        toggleState = state.REVERSE;
    }

    public void stop(){
        leftIntake.set(0.0);
        rightIntake.set(0.0);
        toggleState = state.OFF;
    }
}
