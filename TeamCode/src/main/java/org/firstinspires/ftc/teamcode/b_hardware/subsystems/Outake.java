package org.firstinspires.ftc.teamcode.b_hardware.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Future;

public class Outake extends SubsystemBase {

    private static final double OPEN = 0.27;
    private static final double CLOSED = 0;
    private static final double FLIPPED = 0.49;
    private static final double UNFLIPPED = 0;

    private static final double DIAMETER = 38.0;
    private static final double SPOOL = 185.0;
    private static final double REVOLUTIONS = SPOOL / (DIAMETER * Math.PI);
    private static final double SLIDE_SPEED = 0.05;

    private static final int RED_WHITE = 255;
    private static final int GREEN_WHITE = 255;
    private static final int BLUE_WHITE = 255;
    private static int[] white = new int[] {RED_WHITE, GREEN_WHITE, BLUE_WHITE};

    private static final int RED_YELLOW = 0;
    private static final int GREEN_YELLOW = 255;
    private static final int BLUE_YELLOW = 255;
    private static int[] yellow = new int[] {RED_YELLOW, GREEN_YELLOW, BLUE_YELLOW};

    private static int[][] colors = new int[][] {white, yellow};

    private static final double MARGIN = 0.15;

    private Servo leftFlap, rightFlap, bucket;
    private MotorEx slideMotor;
    private ColorSensor colorSensor;
    private Future<?> closeFlaps;
    private ExecutorService executorService;

    private enum bucketState {
        FLIPPED,
        UNFLIPPED
    }
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

        colorSensor = opMode.hardwareMap.get(ColorSensor.class, "colorSensor");
        closeBucket();

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

    public void closeBucket() {
        closeFlaps = executorService.submit (() -> {
            while (true) {
                for (int[] color : colors) {

                    int red = color[0];
                    int green = color[1];
                    int blue = color[2];

                    double redVal = (double) (colorSensor.red());
                    double greenVal = (double) (colorSensor.green());
                    double blueVal = (double) (colorSensor.blue());

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

    public void runSlides(){
        //TODO: Code this method
        slideMotor.resetEncoder();
        slideMotor.set(SLIDE_SPEED);
        while (slideMotor.encoder.getRevolutions() < REVOLUTIONS) {}
        slideMotor.set(0.0);
    }

}
