package org.firstinspires.ftc.teamcode.b_hardware.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.openftc.revextensions2.ExpansionHubEx;

class Hubs extends SubsystemBase {

    private static ExpansionHubEx controlHub;
    private static ExpansionHubEx expansionHub;

    private static boolean partyMode = false;
    private static int[] rgb = {0, 0, 0};

    public Hubs(OpMode opMode) {
        controlHub = opMode.hardwareMap.get(ExpansionHubEx.class, "Control Hub");
        expansionHub = opMode.hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
    }


    @Override
    public void periodic() {
        if(partyMode) {
            controlHub.setLedColor(rgb[0], rgb[1], rgb[2]);
            expansionHub.setLedColor(rgb[0], rgb[1], rgb[2]);
            if(Math.random() * 100 < 20) {
                rgb[0] = (int) Math.random()*255 +1;
                rgb[1] = (int) Math.random()*255 +1;
                rgb[2] = (int) Math.random()*255 +1;
            }
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



}
