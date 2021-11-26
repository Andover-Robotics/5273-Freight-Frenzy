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
    private static final double OPEN = 0.27;
    private static final double CLOSED = 0;
    private static final double FLIPPED = 0.49;
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
    private static final double SLIDE_STOPPED = 0.0;
//    private static final double RETRACTED    = 537.7 * 0;
//    private static final double LOW_GOAL_POS = 537.7 * 0.5;  // 0.5  rotation of spool
//    private static final double MID_GOAL_POS = 537.7 * 2.0;  // 2.0  rotations
//    private static final double TOP_GOAL_POS = 537.7 * 2.75;  // 2.75 rotations
//    private static final double CAPSTONE_POS = 537.7 * 2.75;  // 2.75 rotations

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


    public Outtake(OpMode opMode, ExecutorService executorService) {
        leftFlap = opMode.hardwareMap.servo.get("leftFlap");
        this.executorService = executorService;
        leftFlap.setDirection(Servo.Direction.FORWARD);
        leftFlap.setPosition(OPEN);

        rightFlap = opMode.hardwareMap.servo.get("rightFlap");
        rightFlap.setDirection(Servo.Direction.FORWARD);
        rightFlap.setPosition(OPEN);

        bucket = opMode.hardwareMap.servo.get("bucketServo");
        bucket.setDirection(Servo.Direction.FORWARD);
        bucket.setPosition(UNFLIPPED);

        /*
        slideMotor = new MotorEx(opMode.hardwareMap, "slideMotor", Motor.GoBILDA.RPM_312);
        slideMotor.setRunMode(Motor.RunMode.VelocityControl);
        slideMotor.motor.setDirection(DcMotorSimple.Direction.FORWARD);

         */

        //leftSensor = opMode.hardwareMap.colorSensor.get("leftBucketSensor");
        //rightSensor = opMode.hardwareMap.colorSensor.get("rightBucketSensor");

        //closeBucket();
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

    /*
    public void runSlides(){
        //TODO: Code this method
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