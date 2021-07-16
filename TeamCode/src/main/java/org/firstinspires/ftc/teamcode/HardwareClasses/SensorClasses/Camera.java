package org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses;

import android.os.Build;

import androidx.annotation.RequiresApi;

import org.firstinspires.ftc.teamcode.HardwareClasses.Robot;
import org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Ew.RingFinderPipeline;
import org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.AimBotPipe;
import org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils;
import org.firstinspires.ftc.teamcode.HardwareClasses.Sensors;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class Camera {
	
	OpenCvWebcam webcam;
	public AimBotPipe aimBot = new AimBotPipe();
	public RingFinderPipeline ringFinder = new RingFinderPipeline();
	public OpenCvPipeline currentPipeline = new OpenCvPipeline() { public Mat processFrame(Mat input) { return null; } };
	
	
	public Camera(OpenCvWebcam webcam){
		this.webcam = webcam;
	}
	
	public void setPipeline(OpenCvPipeline pipeline){
		currentPipeline = pipeline;
		webcam.setPipeline(pipeline);
	}
	
	public void optimizeView(){ webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW); }
	
	public void optimizeEfficiency(){ webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.MAXIMIZE_EFFICIENCY); }
	
	public void startVision(int width, int height) {
		webcam.openCameraDeviceAsync(() -> webcam.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT));
	}
	
	public void stopVision(){ webcam.closeCameraDevice(); }
	
	
	public int startingStackCount(){ return ringFinder.getRingCount(); }
	
	public double highTowerError() {
		if (Sensors.alliance == Sensors.Alliance.BLUE) {
		    return aimBot.getTowerDegreeError();
		}
		return aimBot.getTowerDegreeError();
	}
	
	public double midTowerError() {
		if (Sensors.alliance == Sensors.Alliance.BLUE) {
			return aimBot.getTowerDegreeError();
		}
		return aimBot.getTowerDegreeError();
	}
	
	public double highTowerDistance() {
		if (Sensors.alliance == Sensors.Alliance.BLUE) {
			return aimBot.getDistance2Tower();
		}
		return aimBot.getDistance2Tower();
	}
	
	
	public double midTowerDistance() {
		if (Sensors.alliance == Sensors.Alliance.BLUE) {
			return aimBot.getDistance2Tower();
		}
		return aimBot.getDistance2Tower();
	}
	
	
	public boolean isTowerFound(){ return aimBot.isTowerFound(); }
	
	public double shooterOffsetAngle(){ return aimBot.shooterOffsetAngle(); }
	
	
	
	@RequiresApi(api = Build.VERSION_CODES.N)
	public double leftPSAimAngle(){
		if (Sensors.alliance == Sensors.Alliance.BLUE) {
			return Robot.closestTarget(aimBot.getPSDegreeError(VisionUtils.PowerShot.PS_CLOSE));
		}
		return Robot.closestTarget(aimBot.getPSDegreeError(VisionUtils.PowerShot.PS_CLOSE));
	}
	
	@RequiresApi(api = Build.VERSION_CODES.N)
	public double centerPSAimAngle(){
		return 2;
	}
	
	@RequiresApi(api = Build.VERSION_CODES.N)
	public double rightPSAimAngle(){
		return 2;
	}
	
}
