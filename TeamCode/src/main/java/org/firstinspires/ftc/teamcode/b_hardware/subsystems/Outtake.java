package org.firstinspires.ftc.teamcode.b_hardware.subsystems;


import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Future;


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
    private static final double FLAP_OPEN = 0.26;
    private static final double FLAP_CLOSED = 0.02;
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
    private static final double TOLERANCE = 44;
    public static final int RETRACTED = 0;
    public static final int LOW_GOAL_POS = 226; // ticks
    public static final int MID_GOAL_POS = 377;
    public static final int TOP_GOAL_POS = 690;
    private static final int CAPSTONE_POS = 650; //TODO: tune these values
    public static final int HOOK_CAPSTONE_POS = 176;

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
    private static boolean leftFlapOpen = true;
    private static boolean rightFlapOpen = true;

    // TODO: more optimized way to do color sense stuff, because this is really jank

    private static final double MARGIN = 0.15;

    private Servo leftFlap, rightFlap, bucket;
    private MotorEx slideMotor;
    private ColorSensor bucketSensor;
    private OpMode opMode;


    public Outtake(@NonNull OpMode opMode) {
        this.opMode = opMode;
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

        slideMotor = new MotorEx(opMode.hardwareMap, "slideMotor", Motor.GoBILDA.RPM_312);
        slideMotor.setRunMode(Motor.RunMode.PositionControl);
        slideMotor.motor.setDirection(DcMotorSimple.Direction.FORWARD);
        slideMotor.setPositionTolerance(30);
        slideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        unFlipBucket();
        closeLeftFlap();
        closeRightFlap();
    }

    public void fullyRetract() { // depending on alliance set the flaps to the correct position as well
        unFlipBucket();
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

    @Override
    public void periodic() {
        if (!slideMotor.atTargetPosition()) {
            slideRun = SlideRun.RUNNING;
            if (Math.abs(targetPosition) < Math.abs(slideMotor.getCurrentPosition())) {
                slideMotor.setPositionCoefficient(0.05);
                slideMotor.set(RETRACT_SPEED);
            } else {
                switch (slideState) {
                    case AT_TOP_GOAL:
                        slideMotor.setPositionCoefficient(0.015);
                        break;
                    case AT_MID_GOAL:
                        slideMotor.setPositionCoefficient(0.017);
                        break;
                    case AT_CAPSTONE:
                        slideMotor.setPositionCoefficient(0.15);
                        break;
                    case AT_LOW_GOAL:
                        slideMotor.setPositionCoefficient(0.25);
                }
                slideMotor.set(SLIDE_SPEED);
            }
            opMode.telemetry.addData("atPosition", "no");
        } else {
            if (slideState == SlideState.RETRACTED) {
                slideMotor.stopMotor();
            }
            else {
                slideMotor.set(SLIDE_STOPPED);
            }

            slideRun = SlideRun.HOLDING;
            opMode.telemetry.addData("atPosition", "yes");
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

    public void flipBucket() {
        bucket.setPosition(FLIPPED);
        rightFlap.setPosition(FLAP_DEPOSIT);
        leftFlap.setPosition(FLAP_DEPOSIT);
        bucketState = BucketState.FLIPPED;
    }

    public void unFlipBucket() {
        bucket.setPosition(UNFLIPPED);

        if (GlobalConfig.alliance == GlobalConfig.Alliance.BLUE) {
            openLeftFlap();
            closeRightFlap();
        }
        else {
            closeLeftFlap();
            openRightFlap();
        }

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