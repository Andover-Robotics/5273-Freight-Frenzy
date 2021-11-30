package org.firstinspires.ftc.teamcode.b_hardware.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.checkerframework.checker.units.qual.A;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.ArrayList;

class Hubs extends SubsystemBase {

    private static ExpansionHubEx controlHub;
    private static ExpansionHubEx expansionHub;



    // variables for current draw logic
    private static double leftAvgMilliamps;
    private static double rightAvgMilliamps;
    // TODO: tune milliamp draw for different weight cubes --> put in constants RELATIVE to the avg amps
    private static double LIGHT_WEIGHT_CUBE_MILLIAMP_DRAW = 0.0, MEDIUM_WEIGHT_CUBE_MILLIAMP_DRAW = 0.0, HEAVY_WEIGHT_CUBE_MILLIAMP_DRAW;


    private static boolean partyMode = false;
    private static int[] rgb = {0, 0, 0};

    public Hubs(OpMode opMode) {
        controlHub = opMode.hardwareMap.get(ExpansionHubEx.class, "Control Hub");
        expansionHub = opMode.hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        calibrateIntakeAvgAmperage(opMode);
    }



    @Override
    public void periodic() {
        if(partyMode) {
            controlHub.setLedColor(rgb[0], rgb[1], rgb[2]);
            expansionHub.setLedColor(rgb[0], rgb[1], rgb[2]);
            try {
                Thread.sleep(3000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            rgb[0] = (int) Math.random()*255 +1;
            rgb[1] = (int) Math.random()*255 +1;
            rgb[2] = (int) Math.random()*255 +1;
        }
        else if (!partyMode){
            controlHub.setLedColor(0, 255, 0);
            expansionHub.setLedColor(0, 255, 0);
        }
    }


    public void turnOnPartyMode() {
        partyMode = true;
    }
    public void turnOffPartyMode() {
        partyMode = false;
    }
    public void togglePartyMode() {
        if(partyMode) {
            turnOffPartyMode();
        }
        else if(!partyMode) {
            turnOnPartyMode();
        }
    }

    public double slideCurrentDraw() {
        return controlHub.getMotorCurrentDraw(ExpansionHubEx.CurrentDrawUnits.MILLIAMPS, 4);
    }
    public double leftIntakeCurrentDraw() {
        return controlHub.getMotorCurrentDraw(ExpansionHubEx.CurrentDrawUnits.MILLIAMPS, 1);
    }
    public double rightIntakeCurrentDraw() {
        return controlHub.getMotorCurrentDraw(ExpansionHubEx.CurrentDrawUnits.MILLIAMPS, 2);
    }

// TODO: possible calipration thing durinig init to run the motor and get the current it currently draws and average
//    for more accurate measurements during auto & teleOp also find a way to display it on the driver hub or something
//    BUT -- in the future use the data from the lsat 5 seconds to keep a floating average for better accuracy in cube weight detection

    public void calibrateIntakeAvgAmperage(OpMode opMode) {

        MotorEx leftIntake = new MotorEx(opMode.hardwareMap, "leftIntake", Motor.GoBILDA.RPM_312);
        leftIntake.setRunMode(Motor.RunMode.RawPower);
        leftIntake.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorEx rightIntake = new MotorEx(opMode.hardwareMap, "rightIntake", Motor.GoBILDA.RPM_312);
        rightIntake.setRunMode(Motor.RunMode.RawPower);
        rightIntake.motor.setDirection(DcMotorSimple.Direction.FORWARD);

        final double CALIBRATION_VELOCITY = 1.0;

        ArrayList<Double> leftList = new ArrayList<>();
        ArrayList<Double> rightList = new ArrayList<>();

        leftIntake.set(CALIBRATION_VELOCITY);
        rightIntake.set(CALIBRATION_VELOCITY);
        
        // running the intakes and putting their averages in an arraylist to then get the average;

        for(int i = 0; i < 50000; i++) {
            if(i % 50 == 0) {
                leftList.add(leftIntakeCurrentDraw());
                rightList.add(rightIntakeCurrentDraw());
            }
        }


    }



}
