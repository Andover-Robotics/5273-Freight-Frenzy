package org.firstinspires.ftc.teamcode.a_opmodes.auto.pipeline;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Barcode Tester", group = "Competition")
public class TestBarcodePosition extends OpMode {
    private TSEDetector TSEDetector;

    @Override
    public void init() {
        TSEDetector = new TSEDetector(this, telemetry);
    }

    @Override
    public void loop() {
        TSEDetector.currentlyDetected()
                .ifPresent((pipelineResultDoublePair -> {
                    telemetry.addData("Status", "Looking for a duck");
                    telemetry.addData("Detected", pipelineResultDoublePair.first);
                    telemetry.addData("Confidence", pipelineResultDoublePair.second);
                }));
    }
}
