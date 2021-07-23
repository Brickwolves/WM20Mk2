package org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.OpCuttleFish;

import android.os.Environment;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareClasses.Sensors;
import org.firstinspires.ftc.teamcode.utilities.RingBuffer;
import org.opencv.core.Mat;
import org.opencv.core.MatOfRect;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.CascadeClassifier;
import org.openftc.easyopencv.OpenCvPipeline;

import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Math.round;
import static java.lang.StrictMath.abs;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.curTarget;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.OpCuttleFish.Dash_CuttleFish.CUTTLE_DEBUG_MODE_ON;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.OpCuttleFish.Dash_CuttleFish.bw_thresh;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.OpCuttleFish.Dash_CuttleFish.cuttle_h;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.OpCuttleFish.Dash_CuttleFish.cuttle_w;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.OpCuttleFish.Dash_CuttleFish.cuttle_x;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.OpCuttleFish.Dash_CuttleFish.cuttle_y;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.AXES.X;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.IMG_HEIGHT;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.IMG_WIDTH;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.TOWER_HEIGHT;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.Target.BLUE_GOAL;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.pixels2Degrees;
import static org.opencv.core.Core.mean;
import static org.opencv.imgproc.Imgproc.FONT_HERSHEY_COMPLEX;
import static org.opencv.imgproc.Imgproc.GaussianBlur;
import static org.opencv.imgproc.Imgproc.THRESH_BINARY;
import static org.opencv.imgproc.Imgproc.cvtColor;
import static org.opencv.imgproc.Imgproc.dilate;
import static org.opencv.imgproc.Imgproc.erode;
import static org.opencv.imgproc.Imgproc.findContours;
import static org.opencv.imgproc.Imgproc.line;
import static org.opencv.imgproc.Imgproc.putText;
import static org.opencv.imgproc.Imgproc.rectangle;
import static org.opencv.imgproc.Imgproc.threshold;
//import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
//import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Deprecated.Dash_Shooter.SHOOTER_COEFF;

public class CuttleFish extends OpenCvPipeline {

    // Helper Attributes
    private boolean viewportPaused;
    private ElapsedTime time = new ElapsedTime();

    // Init Regular Mats
    private Mat modified = new Mat();
    private Mat output = new Mat();
    private Mat hierarchy = new Mat();
    private MatOfRect cuttlefish = new MatOfRect();

    // Rectangle settings
    private Scalar orange = new Scalar(252, 186, 3);
    private Scalar lightBlue = new Scalar(3, 252, 227);
    private int thickness = 2;
    private int font = FONT_HERSHEY_COMPLEX;
    private RingBuffer<Double> distanceBuffer = new RingBuffer<Double>(4, 0.0);
    private double distanceSum = 0;


    /*
            Cuttlefish Attributes
                                        */

    private double towerDegreeError = 0;
    private boolean towerFound = false;
    private Rect towerRect = new Rect(0, 0, 0, 0);
    private double towerDistance = 0;

    // cuttlefish.xml
    // cuttlefish_cyclops.xml
    // cuttlefish_body.xml
    // cuttlefish_upper.xml
    String xmlFile = Environment.getExternalStorageDirectory().getPath() + "/cuttlefish_body.xml";
    CascadeClassifier classifier = new CascadeClassifier(xmlFile);


    @Override
    public Mat processFrame(Mat input) {
        return detectingPipe(input);
    }

    public Mat detectingPipe(Mat input){
        System.out.println(xmlFile);

        // Copy to output
        input.copyTo(output);

        // Get height and width
        IMG_HEIGHT = input.rows();
        IMG_WIDTH = input.cols();

        // Convert & Copy to outPut image
        cvtColor(input, modified, Imgproc.COLOR_RGB2GRAY);

        threshold(modified, modified, bw_thresh, 255, THRESH_BINARY);
        //Imgproc.equalizeHist(modified, modified);

        // Find cuttlefish
        classifier.detectMultiScale(modified, cuttlefish);

        // Drawing boxes
        for (Rect rect : cuttlefish.toArray()) {
            Imgproc.rectangle(
                    output,                                               // where to draw the box
                    new Point(rect.x, rect.y),                            // bottom left
                    new Point(rect.x + rect.width, rect.y + rect.height), // top right
                    new Scalar(0, 0, 255),
                    3                                                     // RGB colour
            );
        }

        /*
        // Find contours of goal
        contours = new ArrayList<>();
        findContours(modified, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

        // Check if no goal is found
        if (contours.size() == 0) {
            towerFound = false;
            towerDegreeError = 0;
            towerDistance = 0;
            return output;
        }
        towerFound = true;

        // Retrieve all rects
        List<Rect> rects = new ArrayList<>();
        for (int i=0; i < contours.size(); i++){
            Rect rect = boundingRect(contours.get(i));
            rects.add(rect);
        }

        // Retrieve goal contours and make into one large rectangle
        List<Rect> largest_rects = sortRectsByMaxOption(2, AREA, rects);
        towerRect = mergeRects(largest_rects);
        */

        towerRect = new Rect(cuttle_x, cuttle_y, cuttle_w, cuttle_h);

        // Calculate Center
        int center_x = towerRect.x + round(towerRect.width / 2);
        int center_y = towerRect.y + round(towerRect.height / 2);
        Point center = new Point(center_x, center_y);

        // Calculate Error
        double pixel_error = (IMG_WIDTH / 2) - center_x;
        towerDegreeError = pixels2Degrees(pixel_error, X);
        towerDistance = getDistance2Goal();

        // Logging Shapes and Degree & Pixel Data
        rectangle(output, towerRect, (curTarget == BLUE_GOAL) ? lightBlue : orange, thickness);
        line(output, center, new Point(center_x + pixel_error, center_y), new Scalar(0, 0, 255), thickness);
        Point text_center = new Point(5, IMG_HEIGHT - 50);
        putText(output, "Degree Error: " + towerDegreeError, text_center, font, 0.4, new Scalar(255, 255, 0));
        putText(output, "Pixel Error: " + pixel_error, new Point(5, IMG_HEIGHT - 40), font, 0.4, new Scalar(255, 255, 0));

        if (CUTTLE_DEBUG_MODE_ON) return modified;
        return output;
    }


    public boolean isTowerFound(){
        return towerFound;
    }

    public double getRawTowerDegreeError(){
        return (isTowerFound()) ? towerDegreeError : 0;
    }

    public double getGoalDegreeError(){
        return (isTowerFound()) ? towerDegreeError * 0.8 : 0;
    }

    public double getDistance2Goal() {
        if (!isTowerFound() || towerRect.y == 0) return 0;
        double towerHeight = TOWER_HEIGHT - towerRect.y;
        double theta = (towerHeight / IMG_HEIGHT) * .75;
        double distance = 100/Math.tan(theta) - 0;
        distanceSum = distanceSum + distance - distanceBuffer.getValue(distance);
        return (distanceSum / 4) / (1 + abs(Sensors.gyro.absModAngle() - 90) * .00761);
    }

    public Rect getTowerRect(){
        return towerRect;
    }
}