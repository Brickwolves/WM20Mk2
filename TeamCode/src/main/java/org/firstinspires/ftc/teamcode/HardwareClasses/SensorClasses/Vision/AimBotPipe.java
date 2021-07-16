package org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareClasses.Sensors;
import org.firstinspires.ftc.teamcode.utilities.RingBuffer;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Math.round;
import static java.lang.StrictMath.PI;
import static java.lang.StrictMath.abs;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.BLUE_MAX_HSV;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.BLUE_MIN_HSV;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.DEBUG_MODE_ON;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.INIT_COMPLETED;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.INIT_RECT_SIDELENGTH;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.MARGINS;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.RED_MAX_HSV;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.RED_MIN_HSV;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.blur;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.conversion_factor;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.curTarget;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.dilate_const;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.erode_const;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.goalWidth;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.AXES.X;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.IMG_HEIGHT;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.IMG_WIDTH;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.PS_CENTER_DIST;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.PS_CLOSE_DIST;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.PS_FAR_DIST;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.RECT_OPTION.AREA;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.SHOOTER_OFFSET_DISTANCE;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.TOWER_HEIGHT;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.pixels2Degrees;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.sortRectsByMaxOption;
import static org.firstinspires.ftc.teamcode.utilities.MathUtils.degATan;
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

public class AimBotPipe extends OpenCvPipeline {

    // Helper Attributes
    private boolean viewportPaused;
    private ElapsedTime time = new ElapsedTime();

    // Init Regular Mats
    private Mat modified = new Mat();
    private Mat output = new Mat();
    private Mat hierarchy = new Mat();
    private List<MatOfPoint> contours;

    // Rectangle settings
    private Scalar color = new Scalar(255, 0, 255);
    private int thickness = 2;
    private int font = FONT_HERSHEY_COMPLEX;

    //--------------------------------------------------------------------------

    // Red Tower Attributes
    private double redTowerDegreeError = 0;
    private boolean redTowerFound = false;
    private Rect redTowerRect = new Rect(0, 0, 0, 0);

    // Blue Tower Attributes
    private double blueTowerDegreeError = 0;
    private boolean blueTowerFound = false;
    private Rect blueTowerRect = new Rect(0, 0, 0, 0);

    // When we turn, tower distances are a bit sporadic so we use a ringbuffer to average it out
    // Red Tower Distance Attributes
    private double redTowerDistance = 0;
    private double redDistanceSum = 0;
    private RingBuffer<Double> redDistanceBuffer = new RingBuffer<>(4, 0.0);

    // Blue Tower Distance Attributes
    private double blueTowerDistance = 0;
    private double blueDistanceSum = 0;
    private RingBuffer<Double> blueDistanceBuffer = new RingBuffer<>(4, 0.0);

    // Init mats here so we don't repeat
    private Mat blue_thresh = new Mat();
    private Mat red_thresh = new Mat();


    @Override
    public Mat processFrame(Mat input) {

        output = (!INIT_COMPLETED) ? initPipe(input) : regPipe(input);
        return output;
    }

    /**
     * Gives a target on phone, aim at designated goal, then switch2Regular()
     */
    public void switch2AutoCalibrate(){
        INIT_COMPLETED = false;
    }

    /**
     * After auto calibrate, will show goal target
     */
    public void switch2Regular(){
        INIT_COMPLETED = true;
    }

    public Mat initPipe(Mat input){

        // Copy to output
        input.copyTo(output);

        // Get height and width
        IMG_HEIGHT = input.rows();
        IMG_WIDTH = input.cols();

        // Convert & Copy to outPut image
        cvtColor(input, modified, conversion_factor);

        // Blurring
        GaussianBlur(modified, modified, new Size(blur, blur), 0);

        // Retrieve initRect
        int x = (int) (round(IMG_WIDTH / 2) - round(INIT_RECT_SIDELENGTH/2.0));
        int y = (int) (round(IMG_HEIGHT / 2) - round(INIT_RECT_SIDELENGTH/2.0));
        Rect initRect = new Rect(x, y, INIT_RECT_SIDELENGTH, INIT_RECT_SIDELENGTH);


        /*
            M A K E   S U R E   S U B M A T    W O R K S
         */
        //return modified.submat(initRect);

        // Calc aveHSV w/i initRect
        updateHSV(modified, initRect);

        // Log the rect for driver-placement
        rectangle(output, initRect, color, thickness);

        return output;

    }

    public Mat regPipe(Mat input){

        // Copy to output
        input.copyTo(output);

        // Get height and width
        IMG_HEIGHT = input.rows();
        IMG_WIDTH = input.cols();

        // Convert & Copy to outPut image
        cvtColor(input, modified, conversion_factor);

        // Blurring
        GaussianBlur(modified, modified, new Size(blur, blur), 0);

        // This is where it diverges, let's make a red threshold and a blue threshold
        // Make a for loop

        // Thresholding
        inRange(modified, BLUE_MIN_HSV, BLUE_MAX_HSV, modified);
        //inRange(modified, RED_MIN_HSV, RED_MAX_HSV, red_thresh);

        // Erosion and Dilation
        erode(modified, modified, new Mat(erode_const, erode_const, CV_8U));
        dilate(modified, modified, new Mat(dilate_const, dilate_const, CV_8U));

        // Find contours of goal
        contours = new ArrayList<>();
        findContours(modified, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

        // Check if no goal is found
        if (contours.size() == 0) {
            blueTowerFound = false;
            blueTowerDegreeError = 0;
            blueTowerDistance = 0;
            return output;
        }
        blueTowerFound = true;

        // Retrieve all rects
        List<Rect> rects = new ArrayList<>();
        for (int i=0; i < contours.size(); i++){
            Rect rect = boundingRect(contours.get(i));
            rects.add(rect);
            // Release contours after
            // contours.get(i).release();
        }

        // Retrieve goal contours and make into one large rectangle
        List<Rect> largest_rects = sortRectsByMaxOption(2, AREA, rects);
        blueTowerRect = mergeRects(largest_rects);

        // Calculate Center
        int center_x = blueTowerRect.x + round(blueTowerRect.width / 2);
        int center_y = blueTowerRect.y + round(blueTowerRect.height / 2);
        Point center = new Point(center_x, center_y);

        // Calculate Error
        double pixel_error = (IMG_WIDTH / 2) - center_x;
        blueTowerDegreeError = pixels2Degrees(pixel_error, X);
        blueTowerDistance = getDistance2Tower();

        // Logging Shapes and Degree & Pixel Data
        rectangle(output, blueTowerRect, color, thickness);
        line(output, center, new Point(center_x + pixel_error, center_y), new Scalar(0, 0, 255), thickness);
        Point text_center = new Point(5, IMG_HEIGHT - 50);
        putText(output, "Degree Error: " + blueTowerDegreeError, text_center, font, 0.4, new Scalar(255, 255, 0));
        putText(output, "Pixel Error: " + pixel_error, new Point(5, IMG_HEIGHT - 40), font, 0.4, new Scalar(255, 255, 0));

        // Return altered image
        if (DEBUG_MODE_ON) return modified;
        return output;
    }

    /**
     * Finds average HSV of a square on the image. Adds and subtracts margins accordingly to make
     * upper and lower thresholds for both blue and red goal.
     * @param img
     * @param crop
     */
    public void updateHSV(Mat img, Rect crop){
        img.submat(crop);
        Scalar meanHSV = mean(img);
        for (int i=0; i < 3; i++){
            if (curTarget == VisionUtils.Target.BLUE_GOAL){
                BLUE_MAX_HSV.val[i] = clip(meanHSV.val[i] + MARGINS[i], 0, 255);
                BLUE_MIN_HSV.val[i] = clip(meanHSV.val[i] - MARGINS[i], 0, 255);
            }
            else if (curTarget == VisionUtils.Target.RED_GOAL){
                RED_MAX_HSV.val[i] = clip(meanHSV.val[i] + MARGINS[i], 0, 255);
                RED_MIN_HSV.val[i] = clip(meanHSV.val[i] - MARGINS[i], 0, 255);
            }
        }
        //multTelemetry.addData("HSV", meanHSV.toString());
    }

    public double shooterOffsetAngle(){
        return degATan(SHOOTER_OFFSET_DISTANCE, blueTowerDistance);
    }

    /**
     * TO DO, negate for red
     * @param powerShot
     * @return
     */
    public double getPSDegreeError(VisionUtils.PowerShot powerShot){
        double yDistance = blueTowerDistance * Math.cos((Sensors.gyro.modAngle() + blueTowerDegreeError - 90) * (PI / 180));
        double xDistance = blueTowerDistance * Math.sin((Sensors.gyro.modAngle() + blueTowerDegreeError - 90) * (PI / 180));
        double dDistance;
        switch (powerShot) {
            case PS_CLOSE:
                dDistance = xDistance - PS_CLOSE_DIST;
                break;
            case PS_MIDDLE:
                dDistance = xDistance - PS_CENTER_DIST;
                break;
            case PS_FAR:
                dDistance = xDistance - PS_FAR_DIST;
                break;
            default:
                dDistance = 0;
        }
        return (180 / PI) * Math.atan2(dDistance, yDistance) + 90;
    }

    public boolean isTowerFound(){
        return blueTowerFound;
    }

    public double getRawTowerDegreeError(){
        return (isTowerFound()) ? blueTowerDegreeError : 0;
    }

    public double getTowerDegreeError(){
        return (isTowerFound()) ? blueTowerDegreeError * 0.8 : 0;
    }

    public double getDistance2Tower() {
        if (!isTowerFound() || blueTowerRect.y == 0) return 0;
        double towerHeight = TOWER_HEIGHT - blueTowerRect.y;
        double theta = (towerHeight / IMG_HEIGHT) * .75;
        double distance = 100/Math.tan(theta) - 0;
        blueDistanceSum = blueDistanceSum + distance - blueDistanceBuffer.getValue(distance);
        return blueDistanceSum / 4;
    }

    public Rect getTowerRect(){
        return blueTowerRect;
    }

    private Rect mergeRects(List<Rect> rects) {

        // Return first contour if there is only one
        Rect goalRect = rects.get(0);

        // Extrapolate overarching rectangle if there are two
        if (rects.size() == 2) {

            // Init coords of both rectangles
            Rect left = new Rect(0, 0, 0, 0);
            Rect right = new Rect(0, 0, 0, 0);

            // Get bounding rects of second rectangle
            Rect secondRect = rects.get(1);

            // Check second rect is within goal width
            int diff = abs(goalRect.x - secondRect.x);
            if (diff > goalWidth) return goalRect;

            // Check which side rectangles are on, and calculate surrounding box
            if (goalRect.x < secondRect.x) {
                left.x = goalRect.x;
                left.y = goalRect.y;
                right.x = secondRect.x;
                right.y = secondRect.y;
                right.width = secondRect.width;
                right.height = secondRect.height;
            } else {
                left.x = secondRect.x;
                left.y = secondRect.y;
                right.x = goalRect.x;
                right.y = goalRect.y;
                right.width = goalRect.width;
                right.height = goalRect.height;
            }
            goalRect.x = left.x;
            goalRect.y = left.y;
            goalRect.width = abs(right.x - left.x) + right.width;
            goalRect.height = abs(right.y - left.y) + right.height;
        }

        return goalRect;
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

    @Override
    public void onViewportTapped() {
        viewportPaused =        !viewportPaused;
        if (viewportPaused)     VisionUtils.webcam_front.pauseViewport();
        else                    VisionUtils.webcam_front.resumeViewport();
    }
}