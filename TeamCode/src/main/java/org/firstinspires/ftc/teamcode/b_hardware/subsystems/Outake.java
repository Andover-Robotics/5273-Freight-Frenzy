package org.firstinspires.ftc.teamcode.b_hardware.subsystems;


import android.graphics.Color;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Outake extends SubsystemBase {

    private static final double OPEN = 0.27;
    private static final double CLOSED = 0;
    private static final double FLIPPED = 0.49;
    private static final double UNFLIPPED = 0;

    private static final double DIAMETER = 38.0; // in millimeters
    private static final double SPOOL = 185.0; // in millimeters
    private static final double ROTATIONS = SPOOL / (DIAMETER * Math.PI);
    private static final double SLIDE_SPEED = 0.05;

    private Servo leftFlap, rightFlap, bucket;
    private MotorEx slideMotor;
    private ColorSensor leftSensor, rightSensor;

    private enum bucketState {
        FLIPPED,
        UNFLIPPED
    }
    private enum extensionState {
        EXTENDED,
        RETRACTED
    }
    private extensionState eState = extensionState.RETRACTED;
    private bucketState bState = bucketState.UNFLIPPED;

    public Outake(OpMode opMode) {
        leftFlap = opMode.hardwareMap.servo.get("leftFlap");
        leftFlap.setDirection(Servo.Direction.FORWARD);
        leftFlap.setPosition(OPEN);

        rightFlap = opMode.hardwareMap.servo.get("rightFlap");
        rightFlap.setDirection(Servo.Direction.FORWARD);
        rightFlap.setPosition(OPEN);

        bucket = opMode.hardwareMap.servo.get("bucketServo");
        bucket.setDirection(Servo.Direction.FORWARD);
        bucket.setPosition(UNFLIPPED);

        slideMotor = new MotorEx(opMode.hardwareMap, "slideMotor", Motor.GoBILDA.RPM_312);
        slideMotor.setRunMode(Motor.RunMode.VelocityControl);
        slideMotor.motor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftSensor = opMode.hardwareMap.colorSensor.get("leftBucketSensor");
        rightSensor = opMode.hardwareMap.colorSensor.get("rightBucketSensor");

    }

    public void toggleBucket() {
        if(bState == bucketState.UNFLIPPED) {
            flipBucket();
        }
        else if(bState == bucketState.FLIPPED) {
            unFlipBucket();
        }
    }

    public void flipBucket() {
        bucket.setPosition(FLIPPED);
        bState = bucketState.FLIPPED;
    }
    public void unFlipBucket() {
        bucket.setPosition(UNFLIPPED);
        bState = bucketState.UNFLIPPED;
    }

    //TODO: add sensors to auto detect when minerals enter the bucket - auto close the flaps then
    // can start coding now - sensors will be REV color sensors

    public boolean freightInBucket() {
        if((leftSensor.alpha() < 50 || rightSensor.alpha() < 50)) {
            return true;
        }
        return false;
    }

    public void openLeftFlap() {
        leftFlap.setPosition(OPEN);
    }
    public void closeLeftFlap() {
        leftFlap.setPosition(CLOSED);
    }
    public void openRightFlap() {
        rightFlap.setPosition(OPEN);
    }
    public void closeRightFlap() {
        rightFlap.setPosition(CLOSED);
    }
    public void closeFlaps() {
        closeLeftFlap();
        closeRightFlap();
    }
    public void openFlaps() {
        openLeftFlap();
        openRightFlap();
    }

    public void runSlides(){
        //TODO: Code this method
        slideMotor.resetEncoder();
        while (slideMotor.encoder.getRevolutions() < ROTATIONS) {
            slideMotor.set(SLIDE_SPEED);
            eState = extensionState.EXTENDED;
        }
        slideMotor.set(0.0);
        eState = extensionState.RETRACTED;
    }

}
