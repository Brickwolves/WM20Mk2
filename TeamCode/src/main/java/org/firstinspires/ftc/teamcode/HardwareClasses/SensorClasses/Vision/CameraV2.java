package org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.HardwareClasses.Controller;
import org.firstinspires.ftc.teamcode.HardwareClasses.Sensors;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static org.firstinspires.ftc.teamcode.HardwareClasses.Robot.closestTarget;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.curTarget;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_Sanic.RING_MAX_THRESH;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_Sanic.RING_MIN_THRESH;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.Target.BLUE_GOAL;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.Target.RED_GOAL;
import static org.firstinspires.ftc.teamcode.HardwareClasses.Sensors.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.utilities.Utils.hardwareMap;
import static org.firstinspires.ftc.teamcode.utilities.Utils.multTelemetry;

public class CameraV2 {

	public AimBotPipe aimBotPipe = new AimBotPipe();
	public SanicPipeV2 sanicPipe = new SanicPipeV2();
	private OpenCvCamera webcam;
	private String id;
	private boolean display;
	private boolean runningAutoCal = false;
	private ElapsedTime time = new ElapsedTime();

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
		if (Sensors.alliance == BLUE){
			curTarget = BLUE_GOAL;
			return closestTarget(aimBotPipe.getPSBlueAngle(powerShot));
		}
		else {
			curTarget = RED_GOAL;
			return closestTarget(aimBotPipe.getPSRedAngle(powerShot));
		}
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

	public void calibrateRingDetection(boolean tap){

		// If we tap square, run auto sequence once
		if (tap) runningAutoCal = true;

		// If we haven't started autocal
		if (!runningAutoCal){
			sanicPipe.switch2Regular();
			Dash_Sanic.HAS_SET_ONE_RING_HEIGHT = true;
			time.reset();
		}

		// Start up autocal sequence
		else{
			// First 0.5 seconds autocalibrate [0 -> 0.5]
			// After 0.5, switch to regular and get one ring height for 0.5 [0.5 -> 1]
			// Go back to original state after 1 second autocal sequence
			if (time.seconds() < 5) sanicPipe.switch2AutoCalibrate();
			else if (time.seconds() < 10) {
				Dash_Sanic.HAS_SET_ONE_RING_HEIGHT = false;
				sanicPipe.switch2Regular();
			}
			else runningAutoCal = false;
		}
		multTelemetry.addData("Ring Phase", runningAutoCal);
		multTelemetry.addData("RING_MAX", RING_MAX_THRESH);
		multTelemetry.addData("RING_MIN", RING_MIN_THRESH);
	}
}
