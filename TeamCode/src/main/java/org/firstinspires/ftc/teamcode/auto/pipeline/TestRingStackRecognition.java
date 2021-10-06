/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.auto.pipeline;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

//import com.vuforia.CameraDevice;

@TeleOp(name = "OpenCV Ring Stack Tester")
public class TestRingStackRecognition extends LinearOpMode {

  private OpenCvCamera phoneCam;
  final int ONE_RING_AREA = 100, FOUR_RING_AREA = 400;
  final ArrayList<Rect> finalBounds = new ArrayList<>();

  @Override
  public void runOpMode() {
    /*
     * Instantiate an OpenCvCamera object for the camera we'll be using.
     * In this sample, we're using the phone's internal camera. We pass it a
     * CameraDirection enum indicating whether to use the front or back facing
     * camera, as well as the view that we wish to use for camera monitor (on
     * the RC phone). If no camera monitor is desired, use the alternate
     * single-parameter constructor instead (commented out below)
     */
    int cameraMonitorViewId = hardwareMap.appContext.getResources()
        .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK,
        cameraMonitorViewId);

    // OR...  Do Not Activate the Camera Monitor View
    //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

    /*
     * Open the connection to the camera device
     */
    phoneCam.openCameraDevice();

    /*
     * Specify the image processing pipeline we wish to invoke upon receipt
     * of a frame from the camera. Note that switching pipelines on-the-fly
     * (while a streaming session is in flight) *IS* supported.
     */

    SamplePipeline sp = new SamplePipeline();
    phoneCam.setPipeline(sp);

    /*
     * Tell the camera to start streaming images to us! Note that you must make sure
     * the resolution you specify is supported by the camera. If it is not, an exception
     * will be thrown.
     *
     * Also, we specify the rotation that the camera is used in. This is so that the image
     * from the camera sensor can be rotated such that it is always displayed with the image upright.
     * For a front facing camera, rotation is defined assuming the user is looking at the screen.
     * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
     * away from the user.
     */
    phoneCam.startStreaming(320 * 3, 240 * 3, OpenCvCameraRotation.UPRIGHT);

    /*
     * Wait for the user to press start on the Driver Station
     */
    waitForStart();

    while (opModeIsActive()) {
      /*
       * Send some stats to the telemetry
       */
      telemetry.addData("Frame Count", phoneCam.getFrameCount());
      telemetry.addData("FPS", String.format("%.2f", phoneCam.getFps()));
      telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
      telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
      telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
      telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());

      telemetry.addData("Number of rectangles: ", finalBounds.size());
      synchronized (finalBounds) {
        for (Rect r : finalBounds) {
          if (r == null) {
            continue;
          }
          //check size
          int area = r.height * r.width;

          telemetry.addData("Area: ", area);
          if (area > FOUR_RING_AREA) {
            telemetry.addData("# of rings in stack: ", 4);
          } else if (area > ONE_RING_AREA) {
            telemetry.addData("# of rings in stack: ", 1);
          } else {
            telemetry.addData("No rings in stack", 0);
          }
        }

//        sleep(5000);//I had to.....
      }
      telemetry.update();

      /*
       * NOTE: stopping the stream from the camera early (before the end of the OpMode
       * when it will be automatically stopped for you) *IS* supported. The "if" statement
       * below will stop streaming from the camera when the "A" button on gamepad 1 is pressed.
       */
      if (gamepad1.a) {
        /*
         * IMPORTANT NOTE: calling stopStreaming() will indeed stop the stream of images
         * from the camera (and, by extension, stop calling your vision pipeline). HOWEVER,
         * if the reason you wish to stop the stream early is to switch use of the camera
         * over to, say, Vuforia or TFOD, you will also need to call closeCameraDevice()
         * (commented out below), because according to the Android Camera API documentation:
         *         "Your application should only have one Camera object active at a time for
         *          a particular hardware camera."
         *
         * NB: calling closeCameraDevice() will internally call stopStreaming() if applicable,
         * but it doesn't hurt to call it anyway, if for no other reason than clarity.
         *
         * NB2: if you are stopping the camera stream to simply save some processing power
         * (or battery power) for a short while when you do not need your vision pipeline,
         * it is recommended to NOT call closeCameraDevice() as you will then need to re-open
         * it the next time you wish to activate your vision pipeline, which can take a bit of
         * time. Of course, this comment is irrelevant in light of the use case described in
         * the above "important note".
         */
        phoneCam.stopStreaming();
        //webcam.closeCameraDevice();
      }

      /*
       * The viewport (if one was specified in the constructor) can also be dynamically "paused"
       * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
       * when you need your vision pipeline running, but do not require a live preview on the
       * robot controller screen. For instance, this could be useful if you wish to see the live
       * camera preview as you are initializing your robot, but you no longer require the live
       * preview after you have finished your initialization process; pausing the viewport does
       * not stop running your pipeline.
       *
       * The "if" statements below will pause the viewport if the "X" button on gamepad1 is pressed,
       * and resume the viewport if the "Y" button on gamepad1 is pressed.
       */
      else if (gamepad1.x) {
        phoneCam.pauseViewport();
      } else if (gamepad1.y) {
        phoneCam.resumeViewport();
      }

      /*
       * For the purposes of this sample, throttle ourselves to 10Hz loop to avoid burning
       * excess CPU cycles for no reason. (By default, telemetry is only sent to the DS at 4Hz
       * anyway). Of course in a real OpMode you will likely not want to do this.
       */
      sleep(100);
    }
  }

  /*
   * An example image processing pipeline to be run upon receipt of each frame from the camera.
   * Note that the processFrame() method is called serially from the frame worker thread -
   * that is, a new camera frame will not come in while you're still processing a previous one.
   * In other words, the processFrame() method will never be called multiple times simultaneously.
   *
   * However, the rendering of your processed image to the viewport is done in parallel to the
   * frame worker thread. That is, the amount of time it takes to render the image to the
   * viewport does NOT impact the amount of frames per second that your pipeline can process.
   *
   * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
   * frame worker thread. This should not be a problem in the vast majority of cases. However,
   * if you're doing something weird where you do need it synchronized with your OpMode thread,
   * then you will need to account for that accordingly.
   */
  class SamplePipeline extends OpenCvPipeline {

    /*
     * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
     * highly recommended to declare them here as instance variables and re-use them for
     * each invocation of processFrame(), rather than declaring them as new local variables
     * each time through processFrame(). This removes the danger of causing a memory leak
     * by forgetting to call mat.release(), and it also reduces memory pressure by not
     * constantly allocating and freeing large chunks of memory.
     */
    //estimated ranges based on a diagram I found
    final Scalar lowerRange = new Scalar(0, 50, 220);
    final Scalar upperRange = new Scalar(20, 200, 255);

    final Mat test = new Mat(),
        edgeDetector = new Mat(),
        smoothEdges = new Mat(),
        contourDetector = new Mat();

    @SuppressLint("SdCardPath")
    @Override
    public Mat processFrame(Mat input) {
      /*
       * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
       * will only dereference to the same image for the duration of this particular
       * invocation of this method. That is, if for some reason you'd like to save a copy
       * of this particular frame for later use, you will need to either clone it or copy
       * it to another Mat.
       */

      Imgproc.cvtColor(input, test, Imgproc.COLOR_RGB2HLS);

      Core.inRange(test, lowerRange, upperRange, edgeDetector);
      Imgproc.GaussianBlur(edgeDetector, smoothEdges, new Size(13, 13), 0, 0);
//      Imgproc.Canny(smoothEdges, cannyEdges, 100, 300);

      ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
      Imgproc.findContours(smoothEdges, contours, contourDetector, Imgproc.RETR_TREE,
          Imgproc.CHAIN_APPROX_SIMPLE);//What are RETR_TREE and CHAIN_APPROX_SIMPLE? the docs weren't super helpful

      synchronized (finalBounds) {
        finalBounds.clear();
        for (int i = 0; i < contours.size(); i++) {
          MatOfPoint currContour = contours.get(i);
          MatOfPoint2f tmp = new MatOfPoint2f();
          Imgproc.approxPolyDP(new MatOfPoint2f(currContour.toArray()), tmp, 3,
              true);//what's the epsilon value?
          Rect r = Imgproc.boundingRect(new MatOfPoint(tmp.toArray()));
          addCombineRectangle(finalBounds, r, finalBounds.size() - 1);
        }
        for (Rect t : finalBounds) {
          Imgproc.rectangle(input, t, lowerRange, 2);
        }
      }

      if (phoneCam.getFrameCount() % 100 == 0) {
        // save an image to support external experimentation
        Imgcodecs.imwrite("/sdcard/FIRST/pipe-img.png", input);
        Imgcodecs.imwrite("/sdcard/FIRST/pipe-img-edge.png", smoothEdges);
      }

      /*
       * NOTE: to see how to get data from your pipeline to your OpMode as well as how
       * to change which stage of the pipeline is rendered to the viewport when it is
       * tapped, please see {@link PipelineStageSwitchingExample}
       */
      test.release();
      edgeDetector.release();
      smoothEdges.release();
      contourDetector.release();
      return input;
    }
  }

  static boolean overlaps(Rect a, Rect b) {
    return a.tl().inside(b) || a.br().inside(b) || b.tl().inside(a) || b.br().inside(a);
  }

  static Rect combineRect(Rect a, Rect b) {
    int topY = (int) Math.min(a.tl().y, b.tl().y);
    int leftX = (int) Math.min(a.tl().x, b.tl().x);
    int bottomY = (int) Math.max(a.br().y, b.br().y);
    int rightX = (int) Math.max(a.br().x, b.br().x);
    return new Rect(leftX, topY, rightX - leftX, bottomY - topY);
  }

  static void addCombineRectangle(ArrayList<Rect> list, Rect newRect, int ptr) {
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