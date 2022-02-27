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

public class TSEDetector {

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

  public TSEDetector(OpMode opMode, Telemetry telemetry) {
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

    /*
    final Scalar lowerRange = new Scalar(40, 0, 30);
    final Scalar upperRange = new Scalar(80, 90, 255);
    */

    final Scalar lowerRangeHLS = new Scalar(100, 0, 40);
    final Scalar upperRangeHLS = new Scalar(180, 255, 255);

    final Scalar lowerRangeRGB = new Scalar(0, 0, 0);
    final Scalar upperRangeRGB = new Scalar(5, 5, 5);

    /*

    DUCK DETETCTION CONSTANTS

    //static final double TEAM_SHIPPING_ELEMENT_AREA = 1500;
    static final double TEAM_SHIPPING_ELEMENT_AREA = 1250;

    final double MIDDLE_RIGHT_X = 720;
    final double MIDDLE_LEFT_X = 320;
    final double MIN_Y = 120;
     */

    //TEAM SHIPPING ELEMENT CONSTANTS

    static final double TEAM_SHIPPING_ELEMENT_AREA = 30000;

    final double MIDDLE_RIGHT_X = 640;
    final double MIDDLE_LEFT_X = 280;
    final double MIN_Y = 0;

    final Mat test = new Mat(),
            bitmask = new Mat(),
            edgeDetector = new Mat(),
            smoothEdges = new Mat(),
            bitmaskedImage = new Mat(),
            contourDetector = new Mat();
    final MatOfPoint2f polyDpResult = new MatOfPoint2f();
    final List<Rect> bounds = new ArrayList<>();
    final Size gaussianKernelSize = new Size(9, 9);

    double green;

    @SuppressLint("SdCardPath")
    @Override
    public Mat processFrame(Mat input) {
      Rect potentialDuckArea = new Rect(0, 0, input.width(), input.height());
      Imgproc.rectangle(input, potentialDuckArea, new Scalar(255, 255, 255));
      Imgproc.GaussianBlur(input, smoothEdges, gaussianKernelSize, 0, 0);

      Mat bitmaskImage = Imgcodecs.imread("src/main/java/org/firstinspires/ftc/teamcode/a_opmodes/auto/pipeline/bitmask.jpeg");
      Core.inRange(bitmaskImage, lowerRangeRGB, upperRangeRGB, bitmask);
      Core.bitwise_and(bitmask, smoothEdges, bitmaskedImage);

      Imgproc.cvtColor(bitmaskedImage, test, COLOR_RGB2HLS);
      Core.inRange(test, lowerRangeHLS, upperRangeHLS, edgeDetector);

      ArrayList<MatOfPoint> contours = new ArrayList<>();
      Imgproc.findContours(edgeDetector, contours, contourDetector,
              Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

      extractRectBounds(contours);

      for (Rect t : bounds) {
        Imgproc.rectangle(input, t, lowerRangeHLS, 2);
      }

      result = identifyTSEFromBounds().orElse(null);
      if (saveImageNext) {
        Mat cvt = new Mat();
        Imgproc.cvtColor(input, cvt, COLOR_RGB2HLS);
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

    private Optional<Pair<PipelineResult, Double>> identifyTSEFromBounds() {
      if (bounds.size() == 0) {
        return Optional.of(Pair.create(PipelineResult.NONE, 0.9));
      }

      double minError = bounds.stream().map(Rect::area).max(Comparator.naturalOrder()).get();
      Rect boundingBox = null;

      for (Rect t: bounds) {
        if (Math.abs(TEAM_SHIPPING_ELEMENT_AREA - t.area()) <= minError){
          boundingBox = t;
        }
      }

      assert boundingBox != null;


      if (boundingBox.x <= MIDDLE_LEFT_X) {
        return Optional.of(Pair.create(PipelineResult.LEFT, green));
      }
      else if (boundingBox.x <= MIDDLE_RIGHT_X){
        return Optional.of(Pair.create(PipelineResult.MIDDLE, green));
      }
      else {
        return Optional.of(Pair.create(PipelineResult.RIGHT, green));
      }

    }

    private void extractRectBounds(ArrayList<MatOfPoint> contours) {
      bounds.clear();
      for (MatOfPoint contour : contours) {
        // if polydp fails, switch to a local new MatOfPoint2f();
        Imgproc.approxPolyDP(new MatOfPoint2f(contour.toArray()), polyDpResult, 3, true);
        Rect r = Imgproc.boundingRect(new MatOfPoint(polyDpResult.toArray()));
        if (r.y > MIN_Y && r.area() > TEAM_SHIPPING_ELEMENT_AREA)
          addCombineRectangle(bounds, r, bounds.size() - 1);
      }
    }

    private boolean overlaps(Rect a, Rect b) {
      return a.tl().inside(b) || a.br().inside(b) || b.tl().inside(a) || b.br().inside(a);
    }

    private Rect combineRect(Rect a, Rect b) {
      int topY = (int) Math.min(a.tl().y, b.tl().y);
      int leftX = (int) Math.min(a.tl().x, b.tl().x);
      int bottomY = (int) Math.max(a.br().y, b.br().y);
      int rightX = (int) Math.max(a.br().x, b.br().x);
      return new Rect(leftX, topY, rightX - leftX, bottomY - topY);
    }

    private void addCombineRectangle(List<Rect> list, Rect newRect, int ptr) {
      for (int i = ptr; i >= 0; i--) {
        Rect existing = list.get(i);
        if (overlaps(newRect, existing)) {
          list.remove(i);
          addCombineRectangle(list, combineRect(existing, newRect), i - 1);
          return;
        }
      }
      list.add(newRect);
    }
  }
}