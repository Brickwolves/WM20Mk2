package org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision;

import android.os.Build;

import androidx.annotation.RequiresApi;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.HardwareClasses.Controller;
import org.firstinspires.ftc.teamcode.HardwareClasses.Robot;
import org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Deprecated.BlueAimPipeline;
import org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Deprecated.RedAimPipeline;
import org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Deprecated.RingFinderPipeline;
import org.firstinspires.ftc.teamcode.HardwareClasses.Sensors;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.curTarget;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.Target.BLUE_GOAL;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.Target.BLUE_POWERSHOTS;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.Target.RED_GOAL;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.Target.RED_POWERSHOTS;
import static org.firstinspires.ftc.teamcode.HardwareClasses.Sensors.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.utilities.Utils.hardwareMap;

public class CameraV2 {


	public RingFinderPipeline ringFinderPipeline = new RingFinderPipeline();
	public BlueAimPipeline blueAimPipeline = new BlueAimPipeline();
	public RedAimPipeline redAimPipeline = new RedAimPipeline();
	public OpenCvPipeline currentPipeline = new OpenCvPipeline() { public Mat processFrame(Mat input) { return null; } };


	private OpenCvCamera webcam;
	private AimBotPipe aimBotPipe = new AimBotPipe();
	private String id;
	private boolean display;

	public CameraV2(String id, boolean display2Phone){
		this.id = id;

		// If we enabled display, add the cameraMonitorViewId to the creation of our webcam
		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, id), cameraMonitorViewId);

		// Start streaming
		webcam.openCameraDeviceAsync(() -> webcam.startStreaming(432, 240, OpenCvCameraRotation.UPRIGHT));

		// Set the pipeline depending on id
		if (id.equals("Front Camera")) webcam.setPipeline(aimBotPipe);
		else webcam.setPipeline(ringFinderPipeline);
	}
	public CameraV2(String id){
		this.id = id;

		webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, id));

		// Start streaming
		webcam.openCameraDeviceAsync(() -> webcam.startStreaming(432, 240, OpenCvCameraRotation.UPRIGHT));

		// Set the pipeline depending on id
		if (id.equals("Front Camera")) webcam.setPipeline(aimBotPipe);
		else webcam.setPipeline(ringFinderPipeline);
	}

	public void startVision(int width, int height) { webcam.openCameraDeviceAsync(() -> webcam.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT)); }

	public void stopVision(){ webcam.closeCameraDevice(); }

	public void optimizeView(){ webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW); }
	
	public void optimizeEfficiency(){ webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.MAXIMIZE_EFFICIENCY); }
	
	public int startingStackCount(){ return ringFinderPipeline.getRingCount(); }

	public enum VisionOption {
		DEGREES, DISTANCE
	}

	public double highGoalError() {
	    curTarget = (Sensors.alliance == BLUE) ? BLUE_GOAL : RED_GOAL;
		return aimBotPipe.getTowerDegreeError();
	}
	
	public double midGoalError() {
		curTarget = (Sensors.alliance == BLUE) ? RED_GOAL : BLUE_GOAL;
		return aimBotPipe.getTowerDegreeError();
	}
	
	public double highGoalDistance() {
		curTarget = (Sensors.alliance == BLUE) ? BLUE_GOAL : RED_GOAL;
		return aimBotPipe.getDistance2Tower();
	}
	
	public double midGoalDistance() {
		curTarget = (Sensors.alliance == BLUE) ? RED_GOAL : BLUE_GOAL;
		return aimBotPipe.getDistance2Tower();
	}
	
	
	public boolean isHighGoalFound(){
		curTarget = (Sensors.alliance == BLUE) ? BLUE_GOAL : RED_GOAL;
		return aimBotPipe.isTowerFound();
	}

	public boolean isMidGoalFound(){
		curTarget = (Sensors.alliance == BLUE) ? RED_GOAL : BLUE_GOAL;
		return aimBotPipe.isTowerFound();
	}
	
	public double shooterOffsetAngle(){ return blueAimPipeline.shooterOffsetAngle(); }
	
	public double getPowerShot(VisionUtils.PowerShot powerShot){
		return aimBotPipe.getPSDegreeError(powerShot);
	}


	public void calibrateVision(){
		// Hold to autocalibrate on a color
		switch (Controller.touchSensor.getCount()) {
			case 1:
				curTarget = (Sensors.alliance == BLUE) ? BLUE_GOAL : RED_GOAL;
				aimBotPipe.switch2AutoCalibrate();
				break;

			case 2:
				aimBotPipe.switch2Regular();
				break;

			case 3:
				curTarget = (Sensors.alliance == BLUE) ? RED_GOAL : BLUE_GOAL;
				aimBotPipe.switch2AutoCalibrate();
				break;

			case 4:
				aimBotPipe.switch2Regular();
				break;

			case 5:
				Controller.touchSensor.resetCount();
				break;
		}
	}
}
