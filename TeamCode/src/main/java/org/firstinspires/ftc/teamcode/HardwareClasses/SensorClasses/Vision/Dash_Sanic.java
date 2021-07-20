package org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Scalar;

@Config
public class Dash_Sanic {

    // Thresholding values
    public static Scalar RING_MAX_THRESH = new Scalar(0, 0, 0);
    public static Scalar RING_MIN_THRESH = new Scalar(0, 0, 0);

    // Margins for each value
    public static int YM = 100;
    public static int CrM = 100;
    public static int CbM = 20;

    // Threshold tuning
    public static int blur = 75;
    public static int erode_const = 5;
    public static int dilate_const = 5;
    public static double horizonLineRatio = 0.5;

    public static VisionUtils.Auto AUTO = VisionUtils.Auto.BLUE_LEFT;
    public static int RING_Y = 170;

    // Debugging tools
    public static boolean RING_DEBUG_MODE_ON     = false;
    public static boolean RING_AUTO_CALIBRATE_ON = false;
    public static int RING_INIT_RECT_WIDTH = 20;
    public static int RING_INIT_RECT_HEIGHT = 30;

    public static int BLUE_LEFT_X = 390;
    public static int BLUE_RIGHT_X = 0;
    public static int RED_LEFT_X = ((int) VisionUtils.IMG_WIDTH) - RING_INIT_RECT_WIDTH;
    public static int RED_RIGHT_X = 50;
}