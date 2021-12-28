package org.firstinspires.ftc.teamcode.b_hardware.subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.openftc.revextensions2.ExpansionHubMotor;

public class OuttakeWIP extends SubsystemBase {

    private Servo leftFlap, rightFlap, bucket;
    public   MotorEx slideMotor;

    //color sensor constants
    private static final int ALPHA_THRESHOLD = 50;


    // Motor constants

    private static final int RETRACTED = 0, LOW_GOAL_POS = 226, MID_GOAL_POS = 377, TOP_GOAL_POS = 690, CAPSTONE_POS = 800;  // units in ticks
    private static int targetPosition = RETRACTED;
    private static final double SLIDE_SPEED = 0.3;
    private static final int TOLERANCE = 15;

    private enum SlideState {
        RETRACTED,
        LOW_GOAL,
        MID_GOAL,
        TOP_GOAL,
        CAPSTONE;
    }
    public SlideState slideState = SlideState.RETRACTED;

    private boolean bucketFull = false;

    private static final double RIGHT_OPEN = 0.0, RIGHT_CLOSED = 0.25;
    private static boolean rightFlapOpen = true;
    private static final double LEFT_OPEN = 0.25, LEFT_CLOSED = 0.0;
    private static boolean leftFlapOpen  = true;
    private static final double FLIPPED = 0.45, UNFLIPPED = 0;
    private static boolean bucketFlipped = false;


    public OuttakeWIP(@NonNull OpMode opMode) {

        slideMotor = new MotorEx(opMode.hardwareMap, "slideMotor", Motor.GoBILDA.RPM_312);
        slideMotor.setRunMode(Motor.RunMode.PositionControl);
        slideMotor.setInverted(false);
        slideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        slideMotor.set(SLIDE_SPEED);
        slideMotor.setPositionTolerance(TOLERANCE);
        targetPosition = RETRACTED;

        leftFlap = opMode.hardwareMap.servo.get("leftFlap");
        leftFlap.setDirection(Servo.Direction.FORWARD);
        leftFlap.setPosition(LEFT_OPEN);

        rightFlap = opMode.hardwareMap.servo.get("rightFlap");
        rightFlap.setDirection(Servo.Direction.FORWARD);
        rightFlap.setPosition(RIGHT_OPEN);

        bucket = opMode.hardwareMap.servo.get("bucketServo");
        bucket.setDirection(Servo.Direction.FORWARD);
        bucket.setPosition(UNFLIPPED);

    }

    @Override
    public void periodic() {

        //TODO: make this method better to make it smoothly go to the next slide position and better tolerances

        while(!slideMotor.atTargetPosition()) {
            slideMotor.set((targetPosition - slideMotor.getCurrentPosition()) < 50 ? SLIDE_SPEED/3 : SLIDE_SPEED);
            slideMotor.setTargetPosition(targetPosition);
            checkNewPos();
        }

        slideMotor.stopMotor();
        checkNewPos();

        // end of slide code -- beginning of freight detection
    }

    // TODO: add the code in main teleOP to make this outtake method work +
    //       add the current sensing for cube weight detection
    private void checkNewPos() {
        if(slideState == SlideState.RETRACTED) {
            slideMotor.setTargetPosition(RETRACTED);
        }
        else if(slideState == SlideState.LOW_GOAL) {
            slideMotor.setTargetPosition(LOW_GOAL_POS);
        }
        else if(slideState == SlideState.MID_GOAL) {
            slideMotor.setTargetPosition(MID_GOAL_POS);
        }
        else if(slideState == SlideState.TOP_GOAL) {
            slideMotor.setTargetPosition(TOP_GOAL_POS);
        }
        else if(slideState == SlideState.CAPSTONE) {
            slideMotor.setTargetPosition(CAPSTONE_POS);
        }
    }


    public void openLeftFlap() {
        leftFlap.setPosition(LEFT_OPEN);
        leftFlapOpen = true;
    }
    public void closeLeftFlap() {
        leftFlap.setPosition(LEFT_CLOSED);
        leftFlapOpen = false;
    }
    public void openRightFlap() {
        rightFlap.setPosition(RIGHT_OPEN);
        rightFlapOpen = true;
    }
    public void closeRightFlap() {
        rightFlap.setPosition(RIGHT_CLOSED);
        rightFlapOpen = false;
    }

    public void flipBucket() {
        bucket.setPosition(FLIPPED);
        bucketFlipped = true;
    }
    public void unFlipBucket() {
        bucket.setPosition(UNFLIPPED);
        bucketFlipped = false;
    }
    public void toggleBucket() {
        if(bucketFlipped) {
            unFlipBucket();
        }
        else if(!bucketFlipped) {
            flipBucket();
        }
    }




}
