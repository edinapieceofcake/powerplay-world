/*
 * Copyright (c) 2021 OpenFTC Team
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

package edu.edina.library.vision;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class DetectPolePipeline extends OpenCvPipeline
{
    private Telemetry _telemetry;
    private ArrayList<Double> lastFiveWidths;

    public DetectPolePipeline(Telemetry telemetry)
    {
        _telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void finalize()
    {
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat hsv = new Mat();
        Mat frame = new Mat();

        _telemetry.addData("Mat size", input.size());

        input.copyTo(frame);

        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // Define the range of colors we want to detect
        Scalar lowerRed = new Scalar(0, 100, 100);
        Scalar upperRed = new Scalar(10, 255, 255);

        Scalar lowYellow = new Scalar(20, 70, 180);
        Scalar highYellow = new Scalar(32, 255, 255);

        Scalar lowYellow1 = new Scalar(11, 49, 76);
        Scalar highYellow1 = new Scalar(13, 34, 100);

        // Threshold the image to only get the red colors
        Core.inRange(hsv, lowYellow, highYellow, hsv);

        // Use morphological operators to remove noise
        //Imgproc.erode(hsv, hsv, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5)));
        //Imgproc.dilate(hsv, hsv, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5)));

        // Use morphological operators to isolate the cone
        //Imgproc.dilate(hsv, hsv, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5)));
        //Imgproc.erode(hsv, hsv, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5)));
/*
        // Find the contours of the cone
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(hsv, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        _telemetry.addData("contours", contours.size());
       // Draw the contours on the original image
        if (contours.size() > 0) {
            double maxVal = 0;
            int maxValIdx = 0;

            for (int i = 0; i < contours.size(); i++) {
                double contourArea = Imgproc.contourArea(contours.get(i));
                if (maxVal < contourArea) {
                    maxVal = contourArea;
                    maxValIdx = i;
                }

                _telemetry.addData("Contour Size", "%d, %f", maxValIdx, maxVal);
            }

            Imgproc.drawContours(frame, contours, maxValIdx, new Scalar(0, 0, 255), 2);
            //RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(contours.get(maxValIdx).toArray()));
            Rect corners = Imgproc.boundingRect(contours.get(maxValIdx));
            Imgproc.rectangle(frame, rect.boundingRect(), new Scalar(255, 0, 0), 5);
            //double distance = 723.809 * 1.05 / rect.size.width;
            _telemetry.addData("Rectangle Corners", corners);
            _telemetry.addData("Rotated Rectangle", rect);
            _telemetry.addData("Rotated Rectangle Corners", rect.boundingRect());
            //_telemetry.addData("Distance", distance);
        }

        _telemetry.update();
/*
        // Calculate the center of the cone
        Point center = new Point();
        float[] radius = new float[1];
        for (int i = 0; i < contours.size(); i++) {
            MatOfPoint contour = contours.get(i);
            Moments moments = Imgproc.moments(contour);
            center.x = moments.get_m10() / moments.get_m00();
            center.y = moments.get_m01() / moments.get_m00();
            Imgproc.minEnclosingCircle(contour, center, radius);
        }

        // Calculate the distance to the cone
        double distance = 6 / (2 * Math.tan(radius[0] / frame.width() * 57.3 / 2));

        frame.copyTo(input);

        frame.release();
        */
        //frame.copyTo(input);
        hsv.copyTo(input);
        frame.release();
        hsv.release();
        return input;
    }
}

// Check if a point is within the image frame if (center.x >= 0 && center.x < frame.width() && center.y >= 0 && center.y < frame.height()) { // Draw a circle at the center of the cone Imgproc.circle(frame, center, (int)radius[0], new Scalar(255, 0, 0), 2);
// Draw a line pointing in the direction of the cone Point end = new Point(center.x + radius[0] * Math.cos(distance), center.y + radius[0] * Math.sin(distance)); Imgproc.line(frame, center, end, new Scalar(255, 0, 0), 2); }
// Show the original and thresholded images Imgproc.imshow("Original", frame); Imgproc.imshow("Thresholded", hsv);
// Wait for a key press before continuing if (Imgproc.waitKey(1) >= 0) { break; } } } }

// Release the camera camera.release();
// Close the window Imgproc.destroyAllWindows();
// Print a message to indicate that the program has finished System.out.println("Finished detecting the cone"); } }

// Check if any contours were found if (contours.size() == 0) { System.err.println("No contours found"); continue; }
// Check if multiple contours were found if (contours.size() > 1) { System.err.println("Multiple contours found"); continue; }
// Get the only contour that was found MatOfPoint contour = contours.get(0);
// Calculate the moments of the contour Moments moments = Imgproc.moments(contour);
// Calculate the center of the contour center.x = moments.get_m10() / moments.get_m00(); center.y = moments.get_m01() / moments.get_m00();
// Calculate the radius of the minimum enclosing circle Imgproc.minEnclosingCircle(contour, center, radius);
// Calculate the distance to the cone double distance = 6 / (2 * Math.tan(radius[0] / frame.width() * 57.3 / 2));
// Print the coordinates and distance of the cone System.out.println("Coordinates: (" + center.x + ", " + center.y + ")"); System.out.println("Distance: " + distance + " inches");
// Check if the center of the cone is within the image frame if (center.x >= 0 && center.x < frame.width() && center.y >= 0 && center.y < frame.height()) { // Draw a circle at the center of the cone Imgproc.circle(frame, center, (int)radius[0], new Scalar(255, 0, 0), 2);
// Draw a line pointing in the direction of the cone Point end = new Point(center.x + radius[0] * Math.cos(distance), center.y + radius[0] * Math.sin(distance)); Imgproc.line(frame, center, end, new Scalar(255, 0, 0), 2); }
// Show the original and thresholded images Imgproc.imshow("Original", frame); Imgproc.imshow("Thresholded", hsv);
// Wait for a key press before continuing if (Imgproc.waitKey(1) >= 0) { break; } }
// Release the camera camera.release();
// Close the window Imgproc.destroyAllWindows();
// Print a message to indicate that the program has finished System.out.println("Finished detecting the cone"); } }
// There are 18 lines left.


// Check if a point is within the image frame if (center.x >= 0 && center.x < frame.width() && center.y >= 0 && center.y < frame.height()) { // Draw a circle at the center of the cone Imgproc.circle(frame, center, (int)radius[0], new Scalar(255, 0, 0), 2);
// Draw a line pointing in the direction of the cone Point end = new Point(center.x + radius[0] * Math.cos(distance), center.y + radius[0] * Math.sin(distance)); Imgproc.line(frame, center, end, new Scalar(255, 0, 0), 2); }
// Show the original and thresholded images Imgproc.imshow("Original", frame); Imgproc.imshow("Thresholded", hsv);
// Wait for a key press before continuing if (Imgproc.waitKey(1) >= 0) { break; } }
// Release the camera camera.release();
// Close the window Imgproc.destroyAllWindows();
// Print a message to indicate that the program has finished System.out.println("Finished detecting the cone"); } }
