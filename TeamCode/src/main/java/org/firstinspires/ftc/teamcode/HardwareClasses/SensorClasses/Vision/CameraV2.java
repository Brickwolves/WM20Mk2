package org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.HardwareClasses.Controller;
import org.firstinspires.ftc.teamcode.HardwareClasses.Sensors;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static org.firstinspires.ftc.teamcode.HardwareClasses.Robot.closestTarget;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.curTarget;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.Target.BLUE_GOAL;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.Target.RED_GOAL;
import static org.firstinspires.ftc.teamcode.HardwareClasses.Sensors.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.utilities.Utils.hardwareMap;

public class CameraV2 {

	public AimBotPipe aimBotPipe = new AimBotPipe();
	public SanicPipeV2 sanicPipe = new SanicPipeV2();
	private OpenCvCamera webcam;
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
		else webcam.setPipeline(sanicPipe);
	}
	public CameraV2(String id){
		this.id = id;

		webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, id));

		// Start streaming
		webcam.openCameraDeviceAsync(() -> webcam.startStreaming(432, 240, OpenCvCameraRotation.UPRIGHT));

		// Set the pipeline depending on id
		if (id.equals("Front Camera")) webcam.setPipeline(aimBotPipe);
		else webcam.setPipeline(sanicPipe);
	}

	public void stopVision(){ webcam.closeCameraDevice(); }

	public void optimizeView(){ webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW); }
	
	public void optimizeEfficiency(){ webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.MAXIMIZE_EFFICIENCY); }
	
	public int startingStackCount(){ return sanicPipe.getRingCount(); }

	public enum VisionOption {
		DEGREES, DISTANCE
	}

	public void setTarget(VisionUtils.Target target){
		curTarget = target;
	}

	public double highGoalError() {
	    curTarget = (Sensors.alliance == BLUE) ? BLUE_GOAL : RED_GOAL;
		return aimBotPipe.getGoalDegreeError() + Math.abs(Sensors.gyro.absModAngle() - 90) * .05;
	}
	
	public double midGoalError() {
		curTarget = (Sensors.alliance == BLUE) ? RED_GOAL : BLUE_GOAL;
		return aimBotPipe.getGoalDegreeError() + Math.abs(Sensors.gyro.absModAngle() - 90) * .05;
	}
	
	public double highGoalDistance() {
		curTarget = (Sensors.alliance == BLUE) ? BLUE_GOAL : RED_GOAL;
		return aimBotPipe.getDistance2Goal();
	}
	
	public double midGoalDistance() {
		curTarget = (Sensors.alliance == BLUE) ? RED_GOAL : BLUE_GOAL;
		return aimBotPipe.getDistance2Goal();
	}
	
	
	public boolean isHighGoalFound(){
		curTarget = (Sensors.alliance == BLUE) ? BLUE_GOAL : RED_GOAL;
		return aimBotPipe.isTowerFound();
	}

	public boolean isMidGoalFound(){
		curTarget = (Sensors.alliance == BLUE) ? RED_GOAL : BLUE_GOAL;
		return aimBotPipe.isTowerFound();
	}
	
	public double getPowerShotAngle(VisionUtils.PowerShot powerShot){
		curTarget = (Sensors.alliance == BLUE) ? BLUE_GOAL : RED_GOAL;
		return closestTarget(aimBotPipe.getPSDegreeError(powerShot));
	}


	public void calibrateTowerDetection(){
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

	public void calibrateRingDetection(){
		// Hold to autocalibrate on a color
		switch (Controller.touchSensor.getCount()) {
			case 1:
				sanicPipe.switch2AutoCalibrate();
				break;

			case 2:
				sanicPipe.switch2Regular();
				break;

			case 3:
				Controller.touchSensor.resetCount();
				break;
		}
	}
}
