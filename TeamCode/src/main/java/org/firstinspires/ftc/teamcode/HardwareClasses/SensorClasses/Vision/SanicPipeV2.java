package org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Math.round;
import static java.lang.StrictMath.tan;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.blur;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.dilate_const;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.erode_const;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_Sanic.AUTO;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_Sanic.BLUE_LEFT_X;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_Sanic.BLUE_RIGHT_X;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_Sanic.RED_LEFT_X;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_Sanic.RED_RIGHT_X;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_Sanic.RING_AUTO_CALIBRATE_ON;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_Sanic.RING_DEBUG_MODE_ON;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_Sanic.RING_INIT_RECT_HEIGHT;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_Sanic.RING_INIT_RECT_WIDTH;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_Sanic.RING_MAX_THRESH;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_Sanic.RING_MIN_THRESH;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_Sanic.RING_Y;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_Sanic.horizonLineRatio;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.BACK_WEBCAM_HEIGHT;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.IMG_HEIGHT;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.IMG_WIDTH;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.pixels2Degrees;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.sortRectsByMaxOption;
import static org.opencv.core.Core.inRange;
import static org.opencv.core.Core.mean;
import static org.opencv.core.CvType.CV_8U;
import static org.opencv.imgproc.Imgproc.CHAIN_APPROX_SIMPLE;
import static org.opencv.imgproc.Imgproc.FONT_HERSHEY_COMPLEX;
import static org.opencv.imgproc.Imgproc.GaussianBlur;
import static org.opencv.imgproc.Imgproc.RETR_TREE;
import static org.opencv.imgproc.Imgproc.boundingRect;
import static org.opencv.imgproc.Imgproc.cvtColor;
import static org.opencv.imgproc.Imgproc.dilate;
import static org.opencv.imgproc.Imgproc.erode;
import static org.opencv.imgproc.Imgproc.findContours;
import static org.opencv.imgproc.Imgproc.line;
import static org.opencv.imgproc.Imgproc.putText;
import static org.opencv.imgproc.Imgproc.rectangle;
//import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
//import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Deprecated.Dash_Shooter.SHOOTER_COEFF;

public class SanicPipeV2 extends OpenCvPipeline {

    // Helper Attributes
    private boolean viewportPaused;
    private ElapsedTime time = new ElapsedTime();

    // Init Regular Mats
    private Mat modified = new Mat();
    private Mat output = new Mat();
    private Mat hierarchy = new Mat();
    private List<MatOfPoint> contours;

    // Rectangle settings
    private Scalar red = new Scalar(255, 0, 0);
    private int thickness = 2;
    private int font = FONT_HERSHEY_COMPLEX;


    /*
            RingFinding Attributes
                                        */
    // Ring Count stuff
    private boolean was_ring_height_set = false;
    private int one_ring_height = 0;

    // Constants
    private int ring_count = 0;
    private double degrees_error = 0;
    private boolean ringsFound = false;
    private Rect ringRect = new Rect(0, 0, 0, 0);


    @Override
    public Mat processFrame(Mat input) {
        RING_AUTO_CALIBRATE_ON = (time.seconds() < 1) ? true : false;
        return (RING_AUTO_CALIBRATE_ON) ? autoCalibratePipe(input) : detectingPipe(input);
    }

    /**
     * Gives a target on phone, aim at designated goal, then switch2Regular()
     */
    public void switch2AutoCalibrate(){
        RING_AUTO_CALIBRATE_ON = true;
    }

    /**
     * After auto calibrate, will show goal target
     */
    public void switch2Regular(){
        RING_AUTO_CALIBRATE_ON = false;
    }

    public Mat autoCalibratePipe(Mat input){

        // Copy to output
        input.copyTo(output);

        // Get height and width
        IMG_HEIGHT = input.rows();
        IMG_WIDTH = input.cols();

        // Convert & Copy to outPut image
        cvtColor(input, modified, Imgproc.COLOR_RGB2YCrCb);

        // Blurring
        GaussianBlur(modified, modified, new Size(blur, blur), 0);

        // Retrieve initRect
        int x = 0;
        int y = RING_Y;
        switch (AUTO){
            case BLUE_LEFT:
                x = BLUE_LEFT_X;
                break;
            case BLUE_RIGHT:
                x = BLUE_RIGHT_X;
                break;
            case RED_LEFT:
                x = RED_LEFT_X;
                break;
            case RED_RIGHT:
                x = RED_RIGHT_X;
                break;
        }

        Rect initRect = new Rect(x, y, RING_INIT_RECT_WIDTH, RING_INIT_RECT_HEIGHT);

        // Calc aveHSV w/i initRect
        updateHSV(modified, initRect);

        // Log the rect for driver-placement
        rectangle(output, initRect, red, thickness);

        return output;
    }

    public Mat detectingPipe(Mat input){

        // Copy to output
        input.copyTo(output);
        
        // Get height and width
        IMG_HEIGHT = input.rows();
        IMG_WIDTH = input.cols();

        // Take bottom portion
        double horizonY = (int) IMG_HEIGHT * horizonLineRatio;
        Rect bottomRect = new Rect(new Point(0, horizonY), new Point(IMG_WIDTH, IMG_HEIGHT));
        input = input.submat(bottomRect);

        // Convert & Copy to outPut image
        cvtColor(input, modified, Imgproc.COLOR_RGB2YCrCb);

        // Blurring
        GaussianBlur(modified, modified, new Size(Dash_Sanic.blur, Dash_Sanic.blur), 0);

        // Thresholding
        inRange(modified, RING_MIN_THRESH, RING_MAX_THRESH, modified);

        // Erosion and Dilation
        erode(modified, modified, new Mat(erode_const, erode_const, CV_8U));
        dilate(modified, modified, new Mat(dilate_const, dilate_const, CV_8U));

        // Find contours
        contours = new ArrayList<>();
        findContours(modified, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

        // Retrive all rects
        List<Rect> rects = new ArrayList<>();
        for (int i=0; i < contours.size(); i++){
            Rect rect = boundingRect(contours.get(i));
            rects.add(rect);
        }

        // Check if we have detected any orange objects and assume ring_count is 0
        ring_count = 0;
        if (rects.size() > 0) {

            // Retrieve widest (closest) rect
            List<Rect> widest_rects = sortRectsByMaxOption(3, VisionUtils.RECT_OPTION.WIDTH, rects);
            Rect widest_rect = widest_rects.get(0);
            ringRect = widest_rect;
            ringRect.y += horizonY; // We found a rectangle in half an image, need to offset it when we return a whole image

            // Calculate error
            int center_x = widest_rect.x + (widest_rect.width / 2);
            int center_y = widest_rect.y + (widest_rect.height / 2);
            Point center = new Point(center_x, center_y);
            double pixel_error = (IMG_WIDTH / 2) - center_x;
            degrees_error = pixels2Degrees(pixel_error, VisionUtils.AXES.X);

            // Ring Count
            if (!was_ring_height_set) {
                one_ring_height = (int) 0.5 * ringRect.height;
                was_ring_height_set = true;
            }
            ring_count = (ringRect.height > one_ring_height) ? 4 : 1;

            // Log center
            //String coords = "(" + center_x + ", " + center_y + ")";
            //putText(output, coords, center, font, 0.5, color);

            // Log data on screen
            widest_rect.y += horizonY;
            rectangle(output, widest_rect, red, thickness);
            Point text_center = new Point(5, IMG_HEIGHT - 50);
            putText(output, "Degree Error: " + round(degrees_error), text_center, font, 0.4, new Scalar(255, 255, 0));
            putText(output, "Pixel Error: " + pixel_error, new Point(5, IMG_HEIGHT - 40), font, 0.4, new Scalar(255, 255, 0));
            line(output, center, new Point(center_x + pixel_error, center_y), new Scalar(0, 0, 255), thickness);
        }

        // Release all captures
        input.release();
        releaseAllCaptures();

        // Return altered image
        if (RING_DEBUG_MODE_ON) return modified;
        return output;
    }


    public double getDistance2Ring(){
        double pixelsSubtendedByRing = ringRect.y + ringRect.height;
        double radiansSubtendedByRing = pixelsSubtendedByRing * (0.75 / 240);
        double outputDistance = BACK_WEBCAM_HEIGHT / tan(radiansSubtendedByRing) * 100;
        double correctedDistance =  (1.03855 * outputDistance) + 1.51779;
        return correctedDistance;
    }

    public void releaseAllCaptures(){
        modified.release();
        hierarchy.release();
        if (contours != null){
            for (MatOfPoint cnt : contours){
                cnt.release();
            }
        }
    }

    public int getRingCount(){
        return ring_count;
    }

    public double getRingAngle(){
        return (degrees_error > 0) ? (degrees_error - 10) : (degrees_error + 15);
    }
    

    /**
     * Finds average HSV of a square on the image. Adds and subtracts margins accordingly to make
     * upper and lower thresholds for both blue and red goal.
     * @param img
     * @param crop
     */
    public void updateHSV(Mat img, Rect crop){
        Scalar meanHSV = mean(img.submat(crop));

        // We can tune margins of error for each channel with the following
        int[] YCrCb_margins = {Dash_Sanic.YM, Dash_Sanic.CrM, Dash_Sanic.CbM};

        for (int i=0; i < 3; i++){
            RING_MAX_THRESH.val[i] = clip(round(meanHSV.val[i] + YCrCb_margins[i]), 0, 255);
            RING_MIN_THRESH.val[i] = clip(round(meanHSV.val[i] - YCrCb_margins[i]), 0, 255);
        }
        //multTelemetry.addData("HSV", meanHSV.toString());
    }

}