package org.firstinspires.ftc.teamcode.vision;

import static org.opencv.core.Core.inRange;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

public class SampleDetectionPipeline implements VisionProcessor {
    private Mat imgBlur = new Mat();
    private Mat imgHSV = new Mat();
    private Mat mask = new Mat();
    private Mat imgCanny = new Mat();
    private Mat hierarchy = new Mat();

    private List<MatOfPoint> contours = new ArrayList<>();

    // Variables to store input dimensions
    private int inputX;
    private int inputY;

    // Variables to store contour information
    private int contourAmt;
    private double biggestArea;

    Scalar lower;
    Scalar upper;

    Rect biggestPropRect;
    MatOfPoint biggestContour;

    OpenCvWebcam webcam;

    public enum ALLIANCE {
        RED,
        BLUE
    }

    private ALLIANCE alliance;

    public SampleDetectionPipeline(ALLIANCE alliance) {
        this.alliance = alliance;
    }

    // Method to return the number of contours detected
    public int getContourAmt() {
        return contourAmt;
    }

    // Method to return the width of the input frame
    public int getInputX() {
        return inputX;
    }

    // Method to return the height of the input frame
    public int getInputY() {
        return inputY;
    }

    // Method to return the area of the biggest contour
    public double getBiggestArea() {
        return biggestArea;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        biggestContour = new MatOfPoint();
        biggestPropRect = new Rect();
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        if (frame.empty()) {
            return frame;
        }

        // Store frame dimensions
        inputX = (int) frame.size().width;
        inputY = (int) frame.size().height;

        // Step 1: Blur and convert the frame to HSV
        Imgproc.GaussianBlur(frame, imgBlur, new Size(7, 7), 0);
        Imgproc.cvtColor(imgBlur, imgHSV, Imgproc.COLOR_RGB2HSV);

        // Step 2: Define color bounds (red or blue)
        if (alliance == ALLIANCE.RED) {
            lower = new Scalar(0, 100, 0);     // Red min
            upper = new Scalar(10, 255, 255);  // Red max
        } else {
            lower = new Scalar(100, 45, 0);    // Blue min
            upper = new Scalar(125, 255, 255); // Blue max
        }

        // Step 3: Create a mask based on the selected color
        inRange(imgHSV, lower, upper, mask);

        // Step 4: Detect edges using Canny
        Imgproc.Canny(mask, imgCanny, 50, 50);

        // Step 5: Find contours
        contours.clear();
        Imgproc.findContours(imgCanny, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        contourAmt = contours.size();  // Update the contour count

        if (contours.isEmpty()) {
            return frame;
        }

        // Step 6: Find the biggest contour
        biggestArea = 0;
        for (MatOfPoint cnt : contours) {
            double area = Imgproc.boundingRect(cnt).area();
            if (area > biggestArea) {
                biggestArea = area;
                biggestPropRect = Imgproc.boundingRect(cnt);
                biggestContour = cnt;
            }
        }

        // Step 7: Draw all contours in purple
        Imgproc.drawContours(frame, contours, -1, new Scalar(255, 0, 255), 2);

        // Step 8: Draw bounding box around the biggest contour in red
        Imgproc.rectangle(
                frame,
                new Point(biggestPropRect.x, biggestPropRect.y),
                new Point(biggestPropRect.x + biggestPropRect.width, biggestPropRect.y + biggestPropRect.height),
                new Scalar(0, 0, 255), 4);

        // Step 9: Clean up
        imgCanny.release();
        imgHSV.release();
        imgBlur.release();

        return frame;
    }

    @Override
    public void onDrawFrame(android.graphics.Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // No additional drawing logic needed for this example
    }
}
