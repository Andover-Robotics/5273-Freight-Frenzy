package org.firstinspires.ftc.teamcode.d_util.utilclasses;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class PIDMotorEx {

    private MotorEx motor;

    private int targetPosition = 0;
    private int tolerance = 0;

    private double ks = 0.0, kv = 0.0, ka = 0.0;

    public PIDMotorEx(HardwareMap hardwareMap, String id) {
        this.motor = new MotorEx(hardwareMap, id);
        motor.setRunMode(Motor.RunMode.VelocityControl);
        motor.setInverted(false);
    }
    public PIDMotorEx(HardwareMap hardwareMap, String id, @NonNull Motor.GoBILDA motorType) {
        this.motor = new MotorEx(hardwareMap, id, motorType);
        motor.setRunMode(Motor.RunMode.VelocityControl);
        motor.setInverted(false);
    }
    public PIDMotorEx(HardwareMap hardwareMap, String id, double cpr, double rpm) {
        this.motor = new MotorEx(hardwareMap, id, cpr, rpm);
        motor.setRunMode(Motor.RunMode.VelocityControl);
        motor.setInverted(false);
    }




    public boolean atPos() {
        return Math.abs(motor.getCurrentPosition() - targetPosition) < tolerance;
    }



    // SETTERS & GETTERS

    public void setCoefficients(double ks, double kv, double ka) {
        this.ks = ks;
        this.kv = kv;
        this.ka = ka;
        motor.setFeedforwardCoefficients(ks, kv, ka);
    }

    public double getKs() {
        return ks;
    }
    public double getKv() {
        return kv;
    }
    public double getKa() {
        return ka;
    }

    public double[] getCoefficients() {
        return new double[]{ks, kv, ka};
    }

    public int getTargetPosition() {
        return targetPosition;
    }
    public void setTargetPosition(int targetPosition) {
        this.targetPosition = targetPosition;
    }

    public int getTolerance() {
        return tolerance;
    }
    public void setTolerance(int tolerance) {
        this.tolerance = tolerance;
    }

    public void setInverted(boolean inverted) {
        motor.setInverted(inverted);
    }

    public MotorEx motor() {
        return motor;
    }
}

