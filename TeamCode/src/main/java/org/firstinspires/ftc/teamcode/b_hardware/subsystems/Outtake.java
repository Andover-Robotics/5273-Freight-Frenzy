package org.firstinspires.ftc.teamcode.b_hardware.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Future;

//TODO: use revextensions2 to make load sensing with current draw on motor possibly
// test after scrimmage

public class Outtake extends SubsystemBase {

    // Servo positions for the arms and the bucket
    private static final double RIGHT_OPEN = 0.0;
    private static final double RIGHT_CLOSED = 0.25;
    private static final double LEFT_OPEN = 0.25;
    private static final double LEFT_CLOSED = 0.0;
    private static final double FLIPPED = 0.45;
    private static final double UNFLIPPED = 0;

    private enum BucketState {
        FLIPPED,
        UNFLIPPED
    }
    private BucketState bucketState = BucketState.UNFLIPPED;
    private enum FlapState {
        OPEN,
        CLOSED
    }
    private FlapState flapState = FlapState.OPEN;

    // Slide motor speeds + positions
    private static final double DIAMETER = 38.0;
    private static final double SPOOL = 185.0;
    private static final double ROTATIONS = SPOOL / (DIAMETER * Math.PI);
    private static final double SLIDE_SPEED = 0.05;
//    private static final double SLIDE_STOPPED = 0.0;
    private static final int RETRACTED    =    5;  // 5 all are in ticks
    private static final int LOW_GOAL_POS = -226;  // -226 ticks
    private static final int MID_GOAL_POS = -377;  // -377
    private static final int TOP_GOAL_POS = -690;  // -690
    private static final int CAPSTONE_POS = -800;  // -800 TODO: tune these values

    private enum SlideState {
        RETRACTED,
        AT_LOW_GOAL,
        AT_MID_GOAL,
        AT_TOP_GOAL,
        AT_CAPSTONE
    }
    private SlideState slideState = SlideState.RETRACTED;

    // TODO: more optimized way to do color sense stuff, because this is really jank
    // Color sensing vars for balls
    private static final int RED_WHITE = 255;
    private static final int GREEN_WHITE = 255;
    private static final int BLUE_WHITE = 255;
    private static int[] white = new int[] {RED_WHITE, GREEN_WHITE, BLUE_WHITE};

    //color sensing vars for cubes
    private static final int RED_YELLOW = 0;
    private static final int GREEN_YELLOW = 255;
    private static final int BLUE_YELLOW = 255;
    private static int[] yellow = new int[] {RED_YELLOW, GREEN_YELLOW, BLUE_YELLOW};

    private static int[][] colors = new int[][] {white, yellow};

    private static final double MARGIN = 0.15;

    private Servo leftFlap, rightFlap, bucket;
    private MotorEx slideMotor;
    private ColorSensor leftSensor, rightSensor;
    private ExecutorService executorService;


    public Outtake(OpMode opMode) {
        leftFlap = opMode.hardwareMap.servo.get("leftFlap");
//        this.executorService = executorService;
        leftFlap.setDirection(Servo.Direction.FORWARD);
        leftFlap.setPosition(LEFT_OPEN);

        rightFlap = opMode.hardwareMap.servo.get("rightFlap");
        rightFlap.setDirection(Servo.Direction.FORWARD);
        rightFlap.setPosition(RIGHT_OPEN);

        bucket = opMode.hardwareMap.servo.get("bucketServo");
        bucket.setDirection(Servo.Direction.FORWARD);
        bucket.setPosition(UNFLIPPED);


        slideMotor = new MotorEx(opMode.hardwareMap, "slideMotor", Motor.GoBILDA.RPM_312);
        slideMotor.setRunMode(Motor.RunMode.PositionControl);
        slideMotor.motor.setDirection(DcMotorSimple.Direction.FORWARD);
        slideMotor.motor.setTargetPosition(RETRACTED);



        //leftSensor = opMode.hardwareMap.colorSensor.get("leftBucketSensor");
        //rightSensor = opMode.hardwareMap.colorSensor.get("rightBucketSensor");

        //closeBucket();
    }

    public void updateSlidePos() {
//        slideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        if(slideState == SlideState.RETRACTED) {
//            slideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
            goToPosition(RETRACTED);
        }
        else if(slideState == SlideState.AT_LOW_GOAL) {
            goToPosition(LOW_GOAL_POS);
        }
        else if(slideState == SlideState.AT_MID_GOAL) {
            goToPosition(MID_GOAL_POS);
        }
        else if(slideState == SlideState.AT_TOP_GOAL) {
            goToPosition(TOP_GOAL_POS);
        }
        else if(slideState == SlideState.AT_CAPSTONE) {
            goToPosition(CAPSTONE_POS);
        }
    }

    public void goToPosition(int ticks) {
        slideMotor.setTargetPosition(ticks);
    }


//    public MotorEx getMotor() {
//        return slideMotor;
//    }

    public void fullyRetract() {
        slideState = SlideState.RETRACTED;
        return;
    }
    public void goToLowGoal() {
        slideState = SlideState.AT_LOW_GOAL;
        return;
    }
    public void goToMidGoal() {
        slideState = SlideState.AT_MID_GOAL;
        return;
    }
    public void goToTopGoal() {
        slideState = SlideState.AT_TOP_GOAL;
        return;
    }
    public void goToCapstone() {
        slideState = SlideState.AT_CAPSTONE;
        return;
    }



    public void toggleBucket() {
        if(bucketState == BucketState.UNFLIPPED) {
            flipBucket();
        }
        else if(bucketState == BucketState.FLIPPED) {
            unFlipBucket();
        }
    }

    public void flipBucket() {
        bucket.setPosition(FLIPPED);
        bucketState = BucketState.FLIPPED;
    }
    public void unFlipBucket() {
        bucket.setPosition(UNFLIPPED);
        bucketState = BucketState.UNFLIPPED;
    }

    //TODO: add sensors to auto detect when minerals enter the bucket - auto close the flaps then
    // can start coding now - sensors will be REV color sensors
     /*
    public void closeBucket() {
        Future<?> closeFlaps = executorService.submit(() -> {
            while (true) {
                for (int[] color : colors) {

                    int red = color[0];
                    int green = color[1];
                    int blue = color[2];

                    double redVal = (double) (rightSensor.red());
                    double greenVal = (double) (rightSensor.green());
                    double blueVal = (double) (rightSensor.blue());

                    boolean isRed = red * (1 - MARGIN) <= redVal && redVal <= red * (1 + MARGIN);
                    boolean isGreen = green * (1 - MARGIN) <= greenVal && greenVal <= green * (1 + MARGIN);
                    boolean isBlue = blue * (1 - MARGIN) <= blueVal && blueVal <= blue * (1 + MARGIN);

                    if (isRed && isGreen && isBlue) {
                        closeLeftFlap();
                        closeRightFlap();
                    }
                }
            }
        });
    }
    public boolean freightInBucket() {
        if((leftSensor.alpha() < 50 || rightSensor.alpha() < 50)) {
            return true;
        }
        return false;
    }
     */



    public void openLeftFlap() {
        leftFlap.setPosition(LEFT_OPEN);
    }
    public void closeLeftFlap() {
        leftFlap.setPosition(LEFT_CLOSED);
    }
    public void openRightFlap() {
        rightFlap.setPosition(RIGHT_OPEN);
    }
    public void closeRightFlap() {
        rightFlap.setPosition(RIGHT_CLOSED);
    }

    /* DEPRECATED
    public void runSlides(){
        //
        slideMotor.resetEncoder();
        while (slideMotor.encoder.getRevolutions() < ROTATIONS) {
            slideMotor.set(SLIDE_SPEED);
            eState = extensionState.EXTENDED;
        }
        slideMotor.set(SLIDE_STOPPED);
        eState = extensionState.RETRACTED;
    }

     */

}
