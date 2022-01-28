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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.teamcode.GlobalConfig;

@Config
public class Carousel extends SubsystemBase {
    public final Command cmdRunRed = new InstantCommand(this::runRed, this);
    public final Command cmdRunBlue = new InstantCommand(this::runBlue, this);
    public final Command cmdStopCarousel = new InstantCommand(this::stop, this);

    public static double OPTIMAL_RPM = 531;
    public static final double MOTOR_SPEED = 1150;
    public static final double SPEED_PERCENT = OPTIMAL_RPM / MOTOR_SPEED; //531 rpm


    private final MotorEx motor;
    private final CRServo servo;
    private enum State {
        RED_ON,
        BLUE_ON,
        OFF
    }
    private State runState = State.OFF;


    public Carousel(@NonNull OpMode opMode){
        motor = new MotorEx(opMode.hardwareMap, "carousel", Motor.GoBILDA.RPM_1150);
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motor.setInverted(GlobalConfig.alliance == GlobalConfig.Alliance.RED);
        servo = opMode.hardwareMap.crservo.get("carouselLeft");
        servo.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void runRed() {
        servo.setPower(-1);
        runState = State.RED_ON;
    }

    public void runBlue() {
        motor.set(SPEED_PERCENT);
        runState = State.BLUE_ON;
    }

    public void toggleBlue() {
        if(runState == State.BLUE_ON) {
            stop();
        }
        else if(runState == State.RED_ON) {
            stop();
            runBlue();
        }
        else if(runState == State.OFF) {
            runBlue();
        }
    }
    public void toggleRed() {
        if(runState == State.RED_ON) {
            stop();
        }
        else if(runState == State.BLUE_ON) {
            stop();
            runRed();
        }
        else if(runState == State.OFF) {
            runRed();
        }
    }

    public void stop() {
        //motor.stopMotor();
        servo.setPower(0);
        runState = State.OFF;
    }

    public void runAtRPM(double rpmSpeed) {
        motor.set(rpmSpeed / MOTOR_SPEED);
    }

    public MotorEx getMotor() {
        return motor;
    }


}
