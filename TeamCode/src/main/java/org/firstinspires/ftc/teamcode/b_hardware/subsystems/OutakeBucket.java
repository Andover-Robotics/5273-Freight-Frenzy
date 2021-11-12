package org.firstinspires.ftc.teamcode.b_hardware.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class OutakeBucket {

    private static final double OPEN = 0.27;
    private static final double CLOSED = 0;
    private static final double FLIPPED = 0.49;
    private static final double UNFLIPPED = 0;


    private Servo leftFlap, rightFlap, bucket;

    private enum bucketState {
        FLIPPED,
        UNFLIPPED
    }
    private bucketState bState = bucketState.UNFLIPPED;

    public OutakeBucket(OpMode opMode) {
        leftFlap = opMode.hardwareMap.servo.get("leftFlap");
        leftFlap.setDirection(Servo.Direction.FORWARD);
        leftFlap.setPosition(OPEN);

        rightFlap = opMode.hardwareMap.servo.get("rightFlap");
        rightFlap.setDirection(Servo.Direction.FORWARD);
        rightFlap.setPosition(OPEN);

        bucket = opMode.hardwareMap.servo.get("bucketServo");
        bucket.setDirection(Servo.Direction.FORWARD);
        bucket.setPosition(UNFLIPPED);
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

}
