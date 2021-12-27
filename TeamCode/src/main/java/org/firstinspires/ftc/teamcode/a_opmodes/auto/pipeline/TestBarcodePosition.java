package org.firstinspires.ftc.teamcode.a_opmodes.auto.pipeline;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Barcode Tester", group = "Competition")
public class TestBarcodePosition extends OpMode {
    private DuckDetector duckDetector;

    @Override
    public void init() {
        duckDetector = new DuckDetector(this, telemetry);
    }

    @Override
    public void loop() {
        duckDetector.currentlyDetected()
                .ifPresent((pipelineResultDoublePair -> {
                    telemetry.addData("Camera Status", duckDetector.opened);
                    telemetry.addData("Status", "Looking for a duck");
                    telemetry.addData("Detected", pipelineResultDoublePair.first);
                    telemetry.addData("Confidence", pipelineResultDoublePair.second);
                }));
    }
}
