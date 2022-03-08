package org.firstinspires.ftc.teamcode.a_opmodes.auto.pipeline;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class BarcodeDetector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Position {
        LEFT,
        MIDDLE,
        RIGHT,
        NOT_FOUND
    }
    private Position position;

    static final int SCREEN_WIDTH = 1280; //TODO find dimensions of phone
    static final int SCREEN_HEIGHT = 720;
    static final Rect LEFT_ROI = new Rect(new Point(0, 0), new Point((double) SCREEN_WIDTH/3, SCREEN_HEIGHT));
    static final Rect MID_ROI = new Rect(new Point((double) SCREEN_WIDTH/3, 0), new Point((double) 2*SCREEN_WIDTH/3, SCREEN_HEIGHT));
    static final Rect RIGHT_ROI = new Rect(new Point((double) 2*SCREEN_WIDTH/3, 0), new Point(SCREEN_WIDTH, SCREEN_HEIGHT));

    static final double PERCENT_COLOR_THRESHOLD = 0.0075; //TODO determine percent color threshold


    public BarcodeDetector(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV); //rgb to hsv, rgb isn't great for object detection most of the time
        Scalar lowHSV, highHSV; // lower and upper range of yellow
        lowHSV = new Scalar(23, 50, 70);
        highHSV = new Scalar(32, 255, 255);
//        lowHSV = new Scalar(275,61,43);
//        highHSV = new Scalar(277,45,85);

        //thresholding
        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(LEFT_ROI);
        Mat mid = mat.submat(MID_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        double leftVal = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double midVal = Core.sumElems(mid).val[0] / MID_ROI.area() / 255;
        double rightVal = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        left.release();
        mid.release();
        right.release();

        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Mid raw value", (int) Core.sumElems(mid).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Left percentage", Math.round(leftVal * 100) + "%");
        telemetry.addData("Mid percentage", Math.round(midVal * 100) + "%");
        telemetry.addData("Right percentage", Math.round(rightVal * 100) + "%");

        boolean duckLeft = leftVal > PERCENT_COLOR_THRESHOLD;
        boolean duckMid = midVal > PERCENT_COLOR_THRESHOLD;
        boolean duckRight = rightVal > PERCENT_COLOR_THRESHOLD;

        if (duckLeft) {
            position = Position.LEFT;
            telemetry.addData("Duck position: ", "left");
        } else if (duckMid) {
            position = Position.MIDDLE;
            telemetry.addData("Duck position: ", "mid");
        } else if (duckRight) {
            position = Position.RIGHT;
            telemetry.addData("Duck position: ", "right");
        } else {
            position = Position.NOT_FOUND;
            telemetry.addData("Duck position: ", "not found");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorDuck = new Scalar(255, 0, 0);
        Scalar colorNoDuck = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, LEFT_ROI, position == Position.LEFT ? colorDuck:colorNoDuck);
        Imgproc.rectangle(mat, MID_ROI, position == Position.MIDDLE ? colorDuck:colorNoDuck);
        Imgproc.rectangle(mat, RIGHT_ROI, position == Position.RIGHT ? colorDuck:colorNoDuck);

        return mat;
    }

    /***
     * returns the location of the duck on the barcode, either LEFT, MIDDLE, RIGHT, or NOT_FOUND
     * @return position
     */
    public Position getPosition() {
        return position;
    }
}
