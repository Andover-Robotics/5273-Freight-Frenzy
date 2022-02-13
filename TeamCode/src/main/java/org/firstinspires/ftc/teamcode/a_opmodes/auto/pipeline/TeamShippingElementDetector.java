package org.firstinspires.ftc.teamcode.a_opmodes.auto.pipeline;

import android.annotation.SuppressLint;
import android.util.Log;
import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import static org.opencv.imgproc.Imgproc.COLOR_RGB2HLS;

public class TeamShippingElementDetector {

    public enum PipelineResult {
        LEFT(0),
        MIDDLE(1),
        RIGHT(2),
        NONE(3);

        public final int number;

        PipelineResult(int a) {this.number = a;}
    }

    private final OpenCvCamera camera;
    private final RingDetectionPipeline pipeline = new RingDetectionPipeline();
    private volatile Pair<PipelineResult, Double> result = null;
    private volatile boolean saveImageNext = true;
    private Telemetry telemetry;

    public TeamShippingElementDetector(OpMode opMode, Telemetry telemetry) {
        this.telemetry = telemetry;
        WebcamName camName = opMode.hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(camName);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.setPipeline(pipeline);
                camera.startStreaming(320 * 3, 240 * 3, OpenCvCameraRotation.UPRIGHT);
                telemetry.addData("Camera Status", "Opened");
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error Code", errorCode);
            }
        });
    }

    public void saveImage() {
        saveImageNext = true;
    }

    public Optional<Pair<PipelineResult, Double>> currentlyDetected() {
        return Optional.ofNullable(result);
    }

    public void stop() {
        camera.stopStreaming();
    }

    public void close() {
        camera.stopStreaming();
        camera.closeCameraDevice();
    }

    class RingDetectionPipeline extends OpenCvPipeline {

        final Scalar lowerRange = new Scalar(0, 200, 60);
        final Scalar upperRange = new Scalar(180, 255, 255);

    /*

    DUCK DETETCTION CONSTANTS

    //TODO: Test new values and see if duck detetcted instantly; if not revert to previous value(

    //static final double TEAM_SHIPPING_ELEMENT_AREA = 1500;
    static final double TEAM_SHIPPING_ELEMENT_AREA = 1250;

    final double MIDDLE_RIGHT_X = 720;
    final double MIDDLE_LEFT_X = 320;
    final double MIN_Y = 120;
     */

        //TEAM SHIPPING ELEMENT CONSTANTS

        final double MIDDLE_RIGHT_X = 600;
        final double MIDDLE_LEFT_X = 250;
        final double MIN_Y = 10;

        final Mat test = new Mat(),
                edgeDetector = new Mat(),
                smoothEdges = new Mat(),
                greenMat = new Mat(),
                redMat = new Mat(),
                blueMat = new Mat();
        final MatOfPoint2f polyDpResult = new MatOfPoint2f();
        final List<Rect> bounds = new ArrayList<>();
        final Size gaussianKernelSize = new Size(9, 9);

        private ArrayList rgb = new ArrayList<Mat>();

        @SuppressLint("SdCardPath")
        @Override
        public Mat processFrame(Mat input) {
            Rect leftArea = new Rect(0, 0, input.width() / 3, input.height());
            Rect middleArea = new Rect(input.width() / 3, 0, 2 * input.width() / 3, input.height());
            Rect rightArea = new Rect(input.width() / 3, 0, input.width(), input.height());

            rgb.add(greenMat); rgb.add(blueMat); rgb.add(redMat);

            Imgproc.rectangle(input, leftArea, new Scalar(255, 255, 255));
            Imgproc.GaussianBlur(edgeDetector, smoothEdges, gaussianKernelSize, 0, 0);

            Core.split(input, rgb);
            double greenValueLeft = Core.mean(greenMat).val[-1];

            Imgproc.rectangle(input, rightArea, new Scalar(255, 255, 255));
            Imgproc.GaussianBlur(edgeDetector, smoothEdges, gaussianKernelSize, 0, 0);

            Core.split(input, rgb);
            double greenValueRight = Core.mean(greenMat).val[-1];

            Imgproc.rectangle(input, middleArea, new Scalar(255, 255, 255));
            Imgproc.GaussianBlur(edgeDetector, smoothEdges, gaussianKernelSize, 0, 0);

            Core.split(input, rgb);
            double greenValueMiddle = Core.mean(greenMat).val[-1];

            final double max = Math.max(greenValueLeft, Math.max(greenValueMiddle, greenValueRight));

            if (max == greenValueLeft)
                result = Optional.of(Pair.create(PipelineResult.LEFT, 0.8)).orElse(null);

            else if (max == greenValueRight)
                result = Optional.of(Pair.create(PipelineResult.RIGHT, 0.8)).orElse(null);

            else
                result = Optional.of(Pair.create(PipelineResult.MIDDLE, 0.7)).orElse(null);

            if (saveImageNext) {
                Mat cvt = new Mat();
                Log.i("RingStackDetector", "saving current pipeline image");
                for (Rect r : bounds) {
                    Log.i("RingStackDetector", String.format("result x=%d y=%d width=%d height=%d area=%.2f", r.x, r.y, r.width, r.height, r.area()));
                }
                Imgcodecs.imwrite("/sdcard/FIRST/pipe-img.png", cvt);
                Imgcodecs.imwrite("/sdcard/FIRST/pipe-img-smoothEdges.png", smoothEdges);
                saveImageNext = false;
                cvt.release();
            }
            return input;
        }

}
}