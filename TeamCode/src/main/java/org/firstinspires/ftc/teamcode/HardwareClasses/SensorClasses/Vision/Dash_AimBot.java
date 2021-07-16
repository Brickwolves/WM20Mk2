package org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.Target.BLUE_GOAL;

@Config
public class Dash_AimBot {


    // Conversion
    public static int conversion_factor =  Imgproc.COLOR_BGR2HSV;

    // GOAL FINDER
    public static int MIN_H = 100;
    public static int MAX_H = 120;

    public static int MIN_S = 110;
    public static int MAX_S = 255;

    public static int MIN_V = 90;
    public static int MAX_V = 220;

    public static Scalar RED_MAX_HSV = new Scalar(0, 0, 0);
    public static Scalar RED_MIN_HSV = new Scalar(0, 0, 0);
    public static Scalar BLUE_MAX_HSV = new Scalar(0, 0, 0);
    public static Scalar BLUE_MIN_HSV = new Scalar(0, 0, 0);
    public static Scalar RING_MAX_HSV = new Scalar(0, 0, 0);
    public static Scalar RING_MIN_HSV = new Scalar(0, 0, 0);
    public static int[] MARGINS = {10, 10, 10};

    public static int blur = 5;
    public static int erode_const = 5;
    public static int dilate_const = 5;
    public static int goalWidth = 100;
    public static double horizonLineRatio = 1;

    // Debugging tools
    public static VisionUtils.Target curTarget = BLUE_GOAL;
    public static boolean DEBUG_MODE_ON     = true;
    public static boolean AUTOCALIBRATE_ON        = true;
    public static boolean INIT_COMPLETED    = false;
    public static int INIT_RECT_SIDELENGTH  = 10;


}
