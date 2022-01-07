package org.firstinspires.ftc.teamcode.b_hardware.subsystems;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.openftc.revextensions2.ExpansionHubEx;

import java.util.ArrayList;

public class Hubs extends SubsystemBase {

    private static ExpansionHubEx controlHub;
    private static ExpansionHubEx expansionHub;



    // variables for current draw logic
    private static double leftAvgMilliamps;
    private static double rightAvgMilliamps;

    // TODO: tune milliamp draw for different weight cubes --> put in constants RELATIVE to the avg amps
    private static final double WIFFLE_BALL_MILLIAMP_DRAW = 0.0;
    private static final double LIGHT_WEIGHT_CUBE_MILLIAMP_DRAW = 0.0;
    private static final double MEDIUM_WEIGHT_CUBE_MILLIAMP_DRAW = 0.0;
    private static final double HEAVY_WEIGHT_CUBE_MILLIAMP_DRAW = 0.0;

    private static final double velocity = 0.6;

    private static boolean partyMode = false;
    private static final int[] rgb = {0, 0, 0};

    private MotorEx slideMotor;
    private MotorEx rightIntake;
    private MotorEx leftIntake;

    //ShippingElement shippingElement = ShippingElement.NO_ELEMENT;

    public Hubs(@NonNull OpMode opMode) {

        controlHub = opMode.hardwareMap.get(ExpansionHubEx.class, "Control Hub");
        expansionHub = opMode.hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");

        MotorEx slideMotor = new MotorEx(opMode.hardwareMap, "slideMotor", Motor.GoBILDA.RPM_435);
        slideMotor.setRunMode(Motor.RunMode.PositionControl);
        slideMotor.motor.setDirection(DcMotorSimple.Direction.FORWARD);

        MotorEx leftIntake = new MotorEx(opMode.hardwareMap, "leftIntake", Motor.GoBILDA.RPM_312);
        leftIntake.setRunMode(Motor.RunMode.RawPower);
        leftIntake.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        MotorEx rightIntake = new MotorEx(opMode.hardwareMap, "rightIntake", Motor.GoBILDA.RPM_312);
        rightIntake.setRunMode(Motor.RunMode.RawPower);
        rightIntake.motor.setDirection(DcMotorSimple.Direction.FORWARD);

        rightIntake.set(velocity);
        leftIntake.set(velocity);
        //calibrateSlideMotorAverageAmperage(opMode, slideMotor, (int)Math.pow(10.0, 4.0), 10, false, 0.05, 650);
        //calibrateSlideMotorAverageAmperage(opMode, leftIntake, (int)Math.pow(10.0, 4.0), 10, true, 1.0, null);
        //calibrateSlideMotorAverageAmperage(opMode, rightIntake, (int)Math.pow(10.0, 4.0), 10, true, 1.0, null);

    }


    @Override
    public void periodic() {
        if (partyMode) {

            controlHub.setLedColor(rgb[0], rgb[1], rgb[2]);
            expansionHub.setLedColor(rgb[0], rgb[1], rgb[2]);
            try {
                Thread.sleep(3000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            rgb[0] = (int) (Math.random() * 255) +1;
            rgb[1] = (int) (Math.random() * 255) +1;
            rgb[2] = (int) (Math.random() * 255) +1;
        }
        else {

            /*
            if (getShippingElement() != ShippingElement.NO_ELEMENT){
                controlHub.setLedColor(0, 255, 0);
            }
            else {
                controlHub.setLedColor(255, 0, 0);
            }

             */

        }
    }


    public void turnOnPartyMode() {
        partyMode = true;
    }
    public void turnOffPartyMode() {
        partyMode = false;
    }
    public void togglePartyMode() {
        if (partyMode) {
            turnOffPartyMode();
        }
        else {
            turnOnPartyMode();
        }
    }

    public double slideCurrentDraw() {
        return controlHub.getMotorCurrentDraw(ExpansionHubEx.CurrentDrawUnits.MILLIAMPS, 4);
    }
    public double leftIntakeCurrentDraw() {
        return controlHub.getMotorCurrentDraw(ExpansionHubEx.CurrentDrawUnits.MILLIAMPS, 3);
    }
    public double rightIntakeCurrentDraw() {
        return controlHub.getMotorCurrentDraw(ExpansionHubEx.CurrentDrawUnits.MILLIAMPS, 2);
    }

    /*

    public ShippingElement getShippingElement() {

        if (slideCurrentDraw() >= HEAVY_WEIGHT_CUBE_MILLIAMP_DRAW) {
            shippingElement = ShippingElement.HEAVY_CUBE;
        }
        else if (slideCurrentDraw() >= MEDIUM_WEIGHT_CUBE_MILLIAMP_DRAW){
            shippingElement = ShippingElement.LIGHT_CUBE;
        }
        else if (slideCurrentDraw() > LIGHT_WEIGHT_CUBE_MILLIAMP_DRAW && slideCurrentDraw() < MEDIUM_WEIGHT_CUBE_MILLIAMP_DRAW){
            shippingElement = ShippingElement.LIGHT_CUBE;
        }
        else if (slideCurrentDraw() > WIFFLE_BALL_MILLIAMP_DRAW && slideCurrentDraw() < LIGHT_WEIGHT_CUBE_MILLIAMP_DRAW) {
            shippingElement = ShippingElement.WIFFLE_BALL;
        }
        else {
            shippingElement = ShippingElement.NO_ELEMENT;
        }

        return shippingElement;
    }

     */

    // TODO: calibration thing during init to run the motor and get the current it currently draws and average
//    for more accurate measurements during auto & teleOp also find a way to display it on the driver hub or something
//    BUT -- in the future use the data from the lsat 5 seconds to keep a floating average for better accuracy in cube weight detection

    public void calibrateSlideMotorAverageAmperage(OpMode opMode, MotorEx motor, int readings, int interval, boolean rawPower, double calibrationVelocity, Integer targetPosition) {

        ArrayList<Double> amperageReadings = new ArrayList<>();

        if (!rawPower) {
            motor.setTargetPosition(targetPosition);
        }

        motor.set(calibrationVelocity);

        if (rawPower) {
            for (int i = 0; i < readings; i++) {
                if (i % interval == 0) {
                    amperageReadings.add(slideCurrentDraw());
                }
            }
        }
        else {
            while (!motor.atTargetPosition()) {
                amperageReadings.add(slideCurrentDraw());
            }
            motor.set(0.0);
        }

    }





}
