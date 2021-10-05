package org.firstinspires.ftc.teamcode.auto.pipeline;

import android.annotation.SuppressLint;
import android.util.Log;
import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.apache.commons.math3.distribution.NormalDistribution;
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
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

public class TemplateDetector {//TODO: Change this to control hub

  public enum PipelineResult {
    ZERO(0),
    ONE(1),
    TWO(2),
    THREE(3),
    LEFT(69),
    RIGHT(420);//change to game appropriate
    public final int number;

    PipelineResult(int a){this.number = a;}
  }

  private final OpenCvCamera camera;
  private final TemplatePipeline pipeline = new TemplatePipeline();
  private volatile Pair<PipelineResult, Double> result = null;
  private volatile boolean saveImageNext = true;

  public TemplateDetector(OpMode opMode) {
    int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources()
        .getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
    camera = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK,
        cameraMonitorViewId);
    camera.openCameraDevice();
    camera.setPipeline(pipeline);
    camera.startStreaming(320 * 3, 240 * 3, OpenCvCameraRotation.UPRIGHT);
  }

  public void saveImage() {
    saveImageNext = true;
  }

  public Optional<Pair<PipelineResult, Double>> currentlyDetected() {
    return Optional.ofNullable(result);
  }

  public void close() {
    camera.stopStreaming();
    camera.closeCameraDevice();
  }

  class TemplatePipeline extends OpenCvPipeline {

    final Scalar lowerRange = new Scalar(0, 50, 220);
    final Scalar upperRange = new Scalar(20, 200, 255);

    static final double ONE_RING_AREA = 9950, FOUR_RING_AREA = 19840;
    static final double ST_DEV = 10;
    NormalDistribution one_nd = new NormalDistribution(ONE_RING_AREA, ST_DEV);
    NormalDistribution four_nd = new NormalDistribution(FOUR_RING_AREA, ST_DEV);

    final Mat test = new Mat(),
        edgeDetector = new Mat(),
        smoothEdges = new Mat(),
        contourDetector = new Mat();
    final MatOfPoint2f polyDpResult = new MatOfPoint2f();
    final List<Rect> bounds = new ArrayList<>();
    final Size gaussianKernelSize = new Size(9, 9);

    @SuppressLint("SdCardPath")
    @Override
    public Mat processFrame(Mat input) {

      Imgproc.cvtColor(input, test, Imgproc.COLOR_RGB2HLS);
      Core.inRange(test, lowerRange, upperRange, edgeDetector);
      Imgproc.GaussianBlur(edgeDetector, smoothEdges, gaussianKernelSize, 0, 0);

      ArrayList<MatOfPoint> contours = new ArrayList<>();
      Imgproc.findContours(smoothEdges, contours, contourDetector,
          Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

      extractRectBounds(contours);

      for (Rect t : bounds) {
        Imgproc.rectangle(input, t, lowerRange, 2);
      }

      result = identifyStackFromBounds().orElse(null);
      if (saveImageNext) {
        Mat cvt = new Mat();
        Imgproc.cvtColor(input, cvt, Imgproc.COLOR_RGB2BGR);
        Log.i("TemplateDetector", "saving current pipeline image");
        for (Rect r : bounds) {
          Log.i("TemplateDetector", String.format("result x=%d y=%d width=%d height=%d area=%.2f", r.x, r.y, r.width, r.height, r.area()));
        }
        Imgcodecs.imwrite("/sdcard/FIRST/pipe-img.png", cvt);
        Imgcodecs.imwrite("/sdcard/FIRST/pipe-img-smoothEdges.png", smoothEdges);
        saveImageNext = false;
        cvt.release();
      }
      return input;
    }

    // returns a pair containing verdict and confidence from 0 to 1
    private Optional<Pair<PipelineResult, Double>> identifyStackFromBounds() {
      if (bounds.size() == 0) {
        return Optional.of(Pair.create(PipelineResult.ZERO, 0.7));
      }
      double maxArea = bounds.stream().map(Rect::area).max(Comparator.naturalOrder()).get();
      if (Math.abs(maxArea - ONE_RING_AREA) < Math.abs(maxArea - FOUR_RING_AREA)) {
        return Optional.of(Pair.create(PipelineResult.ONE, 0.8));
      } else {
        return Optional.of(Pair.create(PipelineResult.THREE, 0.8));
      }
//      if (bounds.size() == 1) {
//        Rect r = bounds.get(0);
//        //the way this is right now, ONE_RING_AREA and FOUR_RING_AREA should each represent the peak of
//        //a normal confidence distribution
//        if (Math.abs(ONE_RING_AREA - r.area()) > Math.abs(FOUR_RING_AREA - r.area())) {
//          //one ring
//          double conf = one_nd.density(r.area());
//          //double conf = 1 - (Math.abs(ONE_RING_AREA - r.area())/Math.abs(FOUR_RING_AREA - ONE_RING_AREA));
//          return Optional.of(Pair.create(RingStackResult.ONE, conf));
//        }else{
//          //four rings
//          double conf = four_nd.density(r.area());
//          //double conf = 1 - (Math.abs(FOUR_RING_AREA - r.area())/Math.abs(FOUR_RING_AREA - ONE_RING_AREA));
//          return Optional.of(Pair.create(RingStackResult.FOUR, conf));
//        }
//      } else {
//        //this just chooses the largest rectangle for now since it's probably the most likely... that's what i had in mind originally lol
//        Rect maxRect = new Rect();
//        double maxArea = 0;
//        for (Rect r : bounds) {
//          if (r.area() > maxArea) {
//            maxArea = r.area();
//            maxRect = r;
//          }
//        }
//        Rect r = maxRect;
//        if (Math.abs(ONE_RING_AREA - r.area()) > Math.abs(FOUR_RING_AREA - r.area())) {
//          //one ring
//          double conf = one_nd.density(r.area());
//          //double conf = 1 - (Math.abs(ONE_RING_AREA - r.area())/Math.abs(FOUR_RING_AREA - ONE_RING_AREA));
//          return Optional.of(Pair.create(RingStackResult.ONE, conf));
//        } else {
//          //four rings
//          double conf = four_nd.density(r.area());
//          //double conf = 1 - (Math.abs(FOUR_RING_AREA - r.area())/Math.abs(FOUR_RING_AREA - ONE_RING_AREA));
//          return Optional.of(Pair.create(RingStackResult.FOUR, conf));
//        }
//      }
    }


    //Image Processing stuff

    private void extractRectBounds(ArrayList<MatOfPoint> contours) {
      bounds.clear();
      for (MatOfPoint contour : contours) {
        // if polydp fails, switch to a local new MatOfPoint2f();
        Imgproc.approxPolyDP(new MatOfPoint2f(contour.toArray()), polyDpResult, 3, true);
        Rect r = Imgproc.boundingRect(new MatOfPoint(polyDpResult.toArray()));
        if (r.y > 350 && r.area() > ONE_RING_AREA / 2) addCombineRectangle(bounds, r, bounds.size() - 1);
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
