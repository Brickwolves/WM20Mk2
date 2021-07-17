package org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.Target.BLUE_GOAL;

@Config
public class Dash_AimBot {

    public static Scalar RED_MAX_HSV = new Scalar(0, 0, 0);
    public static Scalar RED_MIN_HSV = new Scalar(0, 0, 0);
    public static Scalar BLUE_MAX_HSV = new Scalar(0, 0, 0);
    public static Scalar BLUE_MIN_HSV = new Scalar(0, 0, 0);
    public static Scalar RING_MAX_HSV = new Scalar(0, 0, 0);
    public static Scalar RING_MIN_HSV = new Scalar(0, 0, 0);
    public static int YM = 50;
    public static int CrM = 15;
    public static int CbM = 15;
    public static int HM = 30;
    public static int SM = 30;
    public static int VM = 30;


    public static int blur = 5;
    public static int erode_const = 5;
    public static int dilate_const = 5;
    public static int goalWidth = 100;

    // Debugging tools
    public static int conversion_factor =  Imgproc.COLOR_RGB2HSV;   // Imgproc.COLOR_RGB2YCrCb
    public static VisionUtils.Target curTarget = BLUE_GOAL;
    public static boolean DEBUG_MODE_ON     = false;
    public static boolean AUTO_CALIBRATE_ON = false;
    public static int INIT_RECT_SIDELENGTH  = 10;


}
