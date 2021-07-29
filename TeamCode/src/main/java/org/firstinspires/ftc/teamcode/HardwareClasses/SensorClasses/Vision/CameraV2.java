package org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.HardwareClasses.Controller;
import org.firstinspires.ftc.teamcode.HardwareClasses.Sensors;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;

import static org.firstinspires.ftc.teamcode.HardwareClasses.Robot.closestTarget;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.BLUE_MAX_THRESH;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.BLUE_MIN_THRESH;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.RED_MAX_THRESH;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.RED_MIN_THRESH;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.curTarget;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_Sanic.HAS_SET_ONE_RING_HEIGHT;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_Sanic.ONE_RING_HEIGHT;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_Sanic.RING_MAX_THRESH;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_Sanic.RING_MIN_THRESH;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.Target.BLUE_GOAL;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.Target.RED_GOAL;
import static org.firstinspires.ftc.teamcode.HardwareClasses.Sensors.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.utilities.Loggers.Dash_Reader.FILE_NAME;
import static org.firstinspires.ftc.teamcode.utilities.Loggers.Dash_Reader.LOG_DIR;
import static org.firstinspires.ftc.teamcode.utilities.Utils.towerGridLogger;
import static org.firstinspires.ftc.teamcode.utilities.Utils.ringGridLogger;
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

		time.reset();
	}
	public CameraV2(String id){
		this.id = id;

		webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, id));

		// Start streaming
		webcam.openCameraDeviceAsync(() -> webcam.startStreaming(432, 240, OpenCvCameraRotation.UPRIGHT));

		// Set the pipeline depending on id
		if (id.equals("Front Camera")) webcam.setPipeline(aimBotPipe);
		else webcam.setPipeline(sanicPipe);

		time.reset();
	}

	public static void reloadThresholds(){
		reloadRingThresholds();
		reloadTowerThresholds();
	}

	public static void reloadRingThresholds(){
		FILE_NAME = "ring.csv";

		String splitBy = ",";
		try
		{
			//parsing a CSV file into BufferedReader class constructor
			BufferedReader br = new BufferedReader(new FileReader(LOG_DIR + "ring.csv"));

			// header
			// hsv max
			// hsv min
			// ycrcb max
			// ycrcb min
			// each row has four items, last one is time


			String headers = br.readLine();
			System.out.println("RING: " + headers);

			// update ring goal max hsv
			String[] max_ring_thresh = br.readLine().split(splitBy);
			for (int i=0; i < max_ring_thresh.length - 1; i++){
				RING_MAX_THRESH.val[i] = Double.parseDouble(max_ring_thresh[i]);
			}

			// update ring goal min hsv
			String[] min_ring_thresh = br.readLine().split(splitBy);
			for (int i=0; i < min_ring_thresh.length - 1; i++){
				RING_MIN_THRESH.val[i] = Double.parseDouble(min_ring_thresh[i]);
			}

		}
		catch (IOException e)
		{
			e.printStackTrace();
		}
	}

	public static void reloadTowerThresholds(){

		FILE_NAME = "log.csv";

		String splitBy = ",";
		try
		{
			//parsing a CSV file into BufferedReader class constructor
			BufferedReader br = new BufferedReader(new FileReader(LOG_DIR + "log.csv"));

			// header
			// hsv max
			// hsv min
			// ycrcb max
			// ycrcb min
			// each row has four items, last one is time


			String headers = br.readLine();
			System.out.println("TOWER: " + headers);

			// update blue goal max hsv
			String[] max_blue_thresh = br.readLine().split(splitBy);
			for (int i=0; i < max_blue_thresh.length - 1; i++){
				BLUE_MAX_THRESH.val[i] = Double.parseDouble(max_blue_thresh[i]);
			}

			// update blue goal min hsv
			String[] min_blue_thresh = br.readLine().split(splitBy);
			for (int i=0; i < min_blue_thresh.length - 1; i++){
				BLUE_MIN_THRESH.val[i] = Double.parseDouble(min_blue_thresh[i]);
			}

			// update red goal max hsv
			String[] max_red_thresh = br.readLine().split(splitBy);
			for (int i=0; i < max_red_thresh.length - 1; i++){
				RED_MAX_THRESH.val[i] = Double.parseDouble(max_red_thresh[i]);
			}

			// update red goal min hsv
			String[] min_red_thresh = br.readLine().split(splitBy);
			for (int i=0; i < min_red_thresh.length - 1; i++){
				RED_MIN_THRESH.val[i] = Double.parseDouble(min_red_thresh[i]);
			}
		}
		catch (IOException e)
		{
			e.printStackTrace();
		}
	}

	public static void writeTowerThreshValues(){

		FILE_NAME = "log.csv";

		String[] headers = {"ONE", "TWO", "THREE"};

		// Add blue max thresh values
		for (int i=0; i < BLUE_MAX_THRESH.val.length - 1; i++){
			towerGridLogger.add(headers[i], BLUE_MAX_THRESH.val[i]);
		}
		towerGridLogger.writeRow();


		// Add blue mainthresh values
		for (int i=0; i < BLUE_MIN_THRESH.val.length - 1; i++){
			towerGridLogger.add(headers[i], BLUE_MIN_THRESH.val[i]);
		}
		towerGridLogger.writeRow();


		// Add RED max thresh values
		for (int i=0; i < RED_MAX_THRESH.val.length - 1; i++){
			towerGridLogger.add(headers[i], RED_MAX_THRESH.val[i]);
		}
		towerGridLogger.writeRow();


		// Add RED mainthresh values
		for (int i=0; i < RED_MIN_THRESH.val.length - 1; i++){
			towerGridLogger.add(headers[i], RED_MIN_THRESH.val[i]);
		}
		towerGridLogger.writeRow();

		towerGridLogger.stop();
	}

	public static void writeRingThreshValues(){

		FILE_NAME = "ring.csv";

		String[] headers = {"ONE", "TWO", "THREE"};

		// Add ring max thresh values
		for (int i=0; i < RING_MAX_THRESH.val.length - 1; i++){
			ringGridLogger.add(headers[i], RING_MAX_THRESH.val[i]);
		}
		ringGridLogger.writeRow();



		// Add ring min thresh values
		for (int i=0; i < RING_MIN_THRESH.val.length - 1; i++){
			ringGridLogger.add(headers[i], RING_MIN_THRESH.val[i]);
		}
		ringGridLogger.writeRow();


		ringGridLogger.stop();
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

	public void saveOneRingHeight(){
		HAS_SET_ONE_RING_HEIGHT = !(time.seconds() < 1);
	}

	public void calibrateRingDetection(){
		switch (Controller.touchSensor.getCount()){
			case 0:
				sanicPipe.switch2Regular();
				Dash_Sanic.HAS_SET_ONE_RING_HEIGHT = true;
				break;
			case 1:
				sanicPipe.switch2AutoCalibrate();
				time.reset();
				break;
			case 2:
				Dash_Sanic.HAS_SET_ONE_RING_HEIGHT = false;
				sanicPipe.switch2Regular();
				if (time.seconds() > 1) {
					Dash_Sanic.HAS_SET_ONE_RING_HEIGHT = true;
				}
				break;
			case 3:
				Controller.touchSensor.resetCount();
				break;
		}

		multTelemetry.addData("Ring Phase", Controller.touchSensor.getCount());
		multTelemetry.addData("RING_COUNT", sanicPipe.getRingCount());
		multTelemetry.addData("RING_MAX", RING_MAX_THRESH);
		multTelemetry.addData("RING_MIN", RING_MIN_THRESH);
	}
}
