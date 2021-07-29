package org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Scalar;

@Config
public class Dash_Sanic {

    // Thresholding values
    public static Scalar RING_MAX_THRESH = new Scalar(0, 0, 0);
    public static Scalar RING_MIN_THRESH = new Scalar(0, 0, 0);

    // Margins for each value
    public static int YM = 60;
    public static int CrM = 80;
    public static int CbM = 20;

    // Threshold tuning
    public static int blur = 75;
    public static int erode_const = 5;
    public static int dilate_const = 5;
    public static double horizonLineRatio = 0.5;

    public static VisionUtils.Auto AUTO = VisionUtils.Auto.BLUE_OUTER;
    public static int RING_Y = 160;

    // Debugging tools
    public static boolean RING_DEBUG_MODE_ON     = false;
    public static boolean RING_AUTO_CALIBRATE_ON = false;
    public static int RING_INIT_RECT_WIDTH = 20;
    public static int RING_INIT_RECT_HEIGHT = 30;
    public static int ONE_RING_HEIGHT = 0;
    public static boolean HAS_SET_ONE_RING_HEIGHT = false;

    // The general starting stack positions
    public static int BLUE_OUTER_X = 370;
    public static int BLUE_INNER_X = 20;
    public static int RED_INNER_X = 400;
    public static int RED_OUTER_X = 70;
}