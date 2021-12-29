package org.firstinspires.ftc.teamcode.b_hardware.subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.usb.serial.RobotUsbDeviceTty;


public class Intake extends SubsystemBase {


    public final Command cmdRunLeft = new InstantCommand(this::runLeft, this);
    public final Command cmdRunRight = new InstantCommand(this::runRight, this);
    public final Command cmdStopIntake = new InstantCommand(this::stop, this);


    public static final double INTAKE_SPEED = 0.6;
    private static final double OUTTAKE_SPEED = 1.0;

    public MotorEx leftIntake;
    public MotorEx rightIntake;

    private double prevLeftVelo, curLeftVelo;
    private double prevRightVelo, curRightVelo;
    private final double INTAKE_DETECT_CONST;
    private final double IS_RUNNING_CONST = 15;
    private boolean leftRunning = false, rightRunning= false;
    private double timeWhenIntake, timeWhenReverse;

    private enum state {
        ON,
        OFF,
        REVERSE,
        LEFT,
        RIGHT
    }

    public state runState = state.OFF;
    public OpMode opMode;
    public Intake(@NonNull OpMode opMode){
        this.opMode = opMode;
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

        //                    ____   <-  that is the constant ration level where intaking gets detected
        INTAKE_DETECT_CONST = 0.783333 * ((leftIntake.getMaxRPM()/60) * leftIntake.getCPR());
    }

    @Override
    public void periodic() {
        updateVeloVals();

        if(wasIntakedLeft() && Math.abs(timeWhenIntake - System.currentTimeMillis()) <= 10) {
            timeWhenIntake = System.currentTimeMillis();
            if (Math.abs(System.currentTimeMillis() - timeWhenIntake) > 200)
            {
                stop();
            }
        }
        else if(Math.abs(System.currentTimeMillis() - timeWhenIntake) > 2000 && wasIntakedLeft()) {
            timeWhenReverse = System.currentTimeMillis();
            if(Math.abs(System.currentTimeMillis() - timeWhenReverse) > 1000) {
                stop();
                timeWhenReverse = -1;
            }
            else {
                reverseLeft();
            }
        }

        if(wasIntakedRight() && Math.abs(timeWhenIntake - System.currentTimeMillis()) <= 10) {
            timeWhenIntake = System.currentTimeMillis();
        }
        else if(Math.abs(System.currentTimeMillis() - timeWhenIntake) > 2000 && wasIntakedLeft()) {
            timeWhenReverse = System.currentTimeMillis();
            if(Math.abs(System.currentTimeMillis() - timeWhenReverse) > 250) {
                stop();
                timeWhenReverse = -1;
            }
            else {
                reverseRight();
            }
        }
        else
        {
            opMode.telemetry.addData("Intake: waiting", true);
        }
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
        leftIntake.set(INTAKE_SPEED);
        rightIntake.set(INTAKE_SPEED);
        leftRunning = true;
        rightRunning = true;
        runState = state.ON;
    }



    public void reverseLeft() {
        leftIntake.set(-OUTTAKE_SPEED);
    }

    public void reverseRight() {
        rightIntake.set(-OUTTAKE_SPEED);
    }

    public void runLeft(){
        leftIntake.set(INTAKE_SPEED);
        leftRunning = true;
    }

    public void runRight(){
        rightIntake.set(INTAKE_SPEED);
        rightRunning = true;
    }

    public void switchIntake(){
        if (leftRunning) {
            runRight();
        }
        else {
            runLeft();
        }
    }

    public void stop(){
        leftIntake.stopMotor();
        rightIntake.stopMotor();
        leftRunning = false;
        rightRunning = false;
        runState = state.OFF;
    }
    public void stopLeft() {
        leftIntake.stopMotor();
        leftRunning = false;
    }
    public void stopRight() {
        rightIntake.stopMotor();
        rightRunning = false;
    }


    private void updateVeloVals() {
        prevRightVelo = curRightVelo;
        prevLeftVelo  = curLeftVelo;

        curLeftVelo = leftIntake.getVelocity();
        curRightVelo = rightIntake.getVelocity();
    }

    public boolean wasIntakedLeft() {
        return curLeftVelo < INTAKE_DETECT_CONST;
    }

    public boolean wasIntakedRight() {
        return curRightVelo < INTAKE_DETECT_CONST;
    }


    public double getCurLeftVelo() { return curLeftVelo; }
    public double getCurRightVelo() { return curRightVelo; }
    public double getPrevLeftVelo() { return prevLeftVelo; }
    public double getPrevRightVelo() { return prevRightVelo; }

    public boolean isLeftIntaking() {
        return leftIntake.getVelocity() < IS_RUNNING_CONST;
    }
    public boolean isRightIntaking() {
        return rightIntake.getVelocity() < IS_RUNNING_CONST;
    }

}
