package org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class BlankPipe extends OpenCvPipeline {
    @Override
    public Mat processFrame(Mat input) {
        return input;
    }
}
