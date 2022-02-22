package org.firstinspires.ftc.teamcode.b_hardware.subsystems;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake extends SubsystemBase {

    public class RunSlides extends CommandBase {
        private final int intendedTarget;
        private final SlideState state;

        public RunSlides(int target, SlideState state) {
            intendedTarget = target;
            this.state = state;
        }

        @Override
        public void initialize() {
            targetPosition = intendedTarget;
            slideState = state;
            slideMotor.setTargetPosition(intendedTarget);
            slideRun = SlideRun.RUNNING;
        }

        @Override
        public void execute() {
            // periodic() should run? no https://docs.ftclib.org/ftclib/command-base/command-system/command-scheduler
        }

        @Override
        public boolean isFinished() {
//            return Math.abs(slideMotor.getCurrentPosition() - targetPosition) < TOLERANCE;
            return slideMotor.atTargetPosition();
        }

        @Override
        public void end(boolean interrupted) {
            slideRun = SlideRun.HOLDING;
        }
    }

    public final Command cmdFlipBucket = new InstantCommand(this::flipBucket, this);
    public final Command cmdUnflipBucket = new InstantCommand(this::unFlipBucket, this);

    public final Command cmdOpenRightFlap = new InstantCommand(this::openRightFlap, this);
    public final Command cmdCloseRightFlap = new InstantCommand(this::closeRightFlap, this);
    public final Command cmdOpenLeftFlap = new InstantCommand(this::openLeftFlap, this);
    public final Command cmdCloseLeftFlap = new InstantCommand(this::closeLeftFlap, this);

    // Servo positions for the arms and the bucket
    //TODO: Make in GlobalConfig
    public static final double FLAP_OPEN = 0.26;
    public static final double FLAP_CLOSED = 0.02;
    private static final double FLAP_DEPOSIT = 0.5;
    private static final double FLAP_HOOK = 0.85;

    private static final double FLIPPED = 0.45;
    private static final double UNFLIPPED = 0.0;

    private enum BucketState {
        FLIPPED,
        UNFLIPPED,
        HOOKING
    }

    private BucketState bucketState = BucketState.UNFLIPPED;

    private enum FlapState {
        OPEN,
        CLOSED
    }

    private FlapState flapState = FlapState.OPEN;

    private static final double SLIDE_SPEED = 0.3;

    private static final double SLIDE_STOPPED = 0.15;
    private static final double RETRACT_SPEED = 0.015;
    private static final double ZERO_SPEED = 0.0;
    private static final double TOLERANCE = 5;
    public static final int RETRACTED = 0;
    public static final int LOW_GOAL_POS = 226; // ticks
    public static final int MID_GOAL_POS = 377;
    public static final int TOP_GOAL_POS = 690;
    private static final int CAPSTONE_POS = 650; //TODO: tune these values
    public static final int HOOK_CAPSTONE_POS = 176;

    private int offset = 0;
    private int kOffset = 0;

    private static int targetPosition;

    public enum SlideState {
        RETRACTED,
        AT_LOW_GOAL,
        AT_MID_GOAL,
        AT_TOP_GOAL,
        AT_CAPSTONE,
        INTAKING_CAPSTONE
    }

    private enum SlideRun {
        RUNNING,
        HOLDING
    }

    private SlideState slideState = SlideState.AT_LOW_GOAL;
    private SlideRun slideRun = SlideRun.HOLDING;
    private boolean leftFlapOpen = true;
    private boolean rightFlapOpen = true;
    private boolean autoFlap = true;

    // TODO: more optimized way to do color sense stuff, because this is really jank

    public final Servo leftFlap;
    public final Servo rightFlap;
    private final Servo bucket;
    private final MotorEx slideMotor;
    private final ColorSensor bucketSensor;


    public Outtake(@NonNull OpMode opMode) {
        leftFlap = opMode.hardwareMap.servo.get("leftFlap");
        leftFlap.setDirection(Servo.Direction.FORWARD);
        leftFlap.setPosition(FLAP_OPEN);

        rightFlap = opMode.hardwareMap.servo.get("rightFlap");
        rightFlap.setDirection(Servo.Direction.REVERSE);
        rightFlap.setPosition(FLAP_OPEN);

        bucket = opMode.hardwareMap.servo.get("bucketServo");
        bucket.setDirection(Servo.Direction.FORWARD);
        bucket.setPosition(UNFLIPPED);

        bucketSensor = opMode.hardwareMap.colorSensor.get("bucketSensor");
        bucketSensor.enableLed(true);

        slideMotor = new MotorEx(opMode.hardwareMap, "slideMotor", Motor.GoBILDA.RPM_312);
        slideMotor.setRunMode(Motor.RunMode.PositionControl);
        slideMotor.setInverted(true);
        slideMotor.setPositionTolerance(40);
        slideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        unFlipBucket();
        closeLeftFlap();
        closeRightFlap();
    }

    @Override
    public void periodic() {
        slideMotor.setTargetPosition(targetPosition + kOffset);

        if (!slideMotor.atTargetPosition()) {
            slideRun = SlideRun.RUNNING;
            if (Math.abs(targetPosition + kOffset) < Math.abs(slideMotor.getCurrentPosition())) {
                slideMotor.setPositionCoefficient(0.05);
                slideMotor.set(RETRACT_SPEED);
            } else {
                slideMotor.set(SLIDE_SPEED);
                switch (slideState) {
                    case AT_TOP_GOAL:
                    case AT_CAPSTONE:
                        slideMotor.setPositionCoefficient(0.01);
                        break;
                    case AT_MID_GOAL:
                        slideMotor.setPositionCoefficient(0.009);
                        break;
                    case AT_LOW_GOAL:
                        slideMotor.setPositionCoefficient(0.008);
                        slideMotor.set(0.5);
                }
            }
        } else {
            if (slideState == SlideState.RETRACTED) {
                slideMotor.stopMotor();
            }
            else {
                slideMotor.set(SLIDE_STOPPED);
            }

            slideRun = SlideRun.HOLDING;
        }

        if(autoFlap && isFreightIn()) {
            closeRightFlap();
            closeLeftFlap();
        }
    }

    public void toggleBucket() {
        if (bucketState == BucketState.UNFLIPPED) {
            flipBucket();
        }
        else if (bucketState == BucketState.FLIPPED) {
            unFlipBucket();
        }
    }

    public boolean isFreightIn() {
        return bucketSensor.alpha() > 2500;
    }

    public void fullyRetract() { // depending on alliance set the flaps to the correct position as well
        unFlipBucket();
        resetOffset();
        targetPosition = RETRACTED;
        slideMotor.setTargetPosition(RETRACTED);
        slideRun = SlideRun.RUNNING;
        slideState = SlideState.RETRACTED;
    }

    public void goToLowGoal() {
        closeRightFlap();
        closeLeftFlap();
        leftFlap.close();
        targetPosition = LOW_GOAL_POS;
        slideState = SlideState.AT_LOW_GOAL;
        slideMotor.setTargetPosition(LOW_GOAL_POS);
        slideRun = SlideRun.RUNNING;
    }

    public void goToMidGoal() {
        closeRightFlap();
        closeLeftFlap();
        leftFlap.close();
        targetPosition = MID_GOAL_POS;
        slideState = SlideState.AT_MID_GOAL;
        slideMotor.setTargetPosition(MID_GOAL_POS);
        slideRun = SlideRun.RUNNING;
    }

    public void goToTopGoal() {
        closeRightFlap();
        closeLeftFlap();
        leftFlap.close();
        targetPosition = TOP_GOAL_POS;
        slideState = SlideState.AT_TOP_GOAL;
        slideMotor.setTargetPosition(TOP_GOAL_POS);
        slideRun = SlideRun.RUNNING;
    }

    public void goToCapstone() {
        slideState = SlideState.AT_CAPSTONE;
        slideMotor.setTargetPosition(CAPSTONE_POS);
        slideRun = SlideRun.RUNNING;
    }

    public void addOffset(double offset) {
        this.offset += (int) (offset * 15);
        if(targetPosition == RETRACTED) kOffset = 0;
        else if(targetPosition == CAPSTONE_POS && this.offset > 10) kOffset = 10;
        else if(this.offset >= 90) kOffset = 90;
        else if(this.offset <= -90) kOffset = -90;
        else this.kOffset = this.offset;
    }

    public void resetOffset() {
        offset = 0;
        kOffset = 0;
    }

    public void hookCapstone() {
        slideState = SlideState.INTAKING_CAPSTONE;
        slideMotor.setTargetPosition(HOOK_CAPSTONE_POS);
        slideRun = SlideRun.RUNNING;
    }

    public void extendRightHook() {
        rightFlap.setPosition(FLAP_HOOK);
        bucketState = BucketState.HOOKING;
        rightFlapOpen = true;
    }

    public void extendLeftHook() {
        leftFlap.setPosition(FLAP_HOOK);
        bucketState = BucketState.HOOKING;
        leftFlapOpen = true;
    }

    public void autoRun() {
        System.out.println("autoRun Doing stuff");
        while (true) {
            if (Math.abs(slideMotor.getCurrentPosition() - targetPosition) < TOLERANCE) {
                slideMotor.set(SLIDE_STOPPED);
                slideRun = SlideRun.HOLDING;
                return;
            } else {
                if (Math.abs(targetPosition) < Math.abs(slideMotor.getCurrentPosition())) {
                    slideMotor.set(RETRACT_SPEED);
                } else {
                    slideMotor.setPositionCoefficient(0.05);
                    slideMotor.set(SLIDE_SPEED);
                }
                slideRun = SlideRun.RUNNING;
            }
        }
    }

    public void flipBucket() {
        rightFlap.setPosition(FLAP_DEPOSIT);
        leftFlap.setPosition(FLAP_DEPOSIT);
        bucket.setPosition(FLIPPED);
        bucketState = BucketState.FLIPPED;
    }

    public void unFlipBucket() {
        bucket.setPosition(UNFLIPPED);
        closeLeftFlap();
        closeRightFlap();
        bucketState = BucketState.UNFLIPPED;
    }

    public void openLeftFlap() {
        leftFlap.setPosition(FLAP_OPEN);
        leftFlapOpen = true;
    }

    public void closeLeftFlap() {
        leftFlap.setPosition(FLAP_CLOSED);
        leftFlapOpen = false;
    }

    public void openRightFlap() {
        rightFlap.setPosition(FLAP_OPEN);
        rightFlapOpen = true;
    }

    public void closeRightFlap() {
        rightFlap.setPosition(FLAP_CLOSED);
        rightFlapOpen = false;
    }

    public void toggleLeftFlap() {
        if (leftFlapOpen) {
            closeLeftFlap();
        } else {
            openLeftFlap();
        }
    }

    public void toggleRightFlap() {
        if (rightFlapOpen) {
            closeRightFlap();
        } else {
            openRightFlap();
        }
    }

    public void toggleAutoFlap() {
        autoFlap = !autoFlap;
    }

    public void setAutoFlap(boolean on) {
        autoFlap = on;
    }

    public boolean isAutoFlap() {
        return autoFlap;
    }


}