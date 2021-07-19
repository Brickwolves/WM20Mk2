
package org.firstinspires.ftc.teamcode.Autonomous;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareClasses.Controller;
import org.firstinspires.ftc.teamcode.HardwareClasses.Intake;
import org.firstinspires.ftc.teamcode.HardwareClasses.Robot;
import org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.PowerShot;
import org.firstinspires.ftc.teamcode.HardwareClasses.Sensors;
import org.firstinspires.ftc.teamcode.HardwareClasses.Shooter;
import org.firstinspires.ftc.teamcode.HardwareClasses.Wobble;
import org.firstinspires.ftc.teamcode.utilities.Utils;

import static android.os.SystemClock.sleep;
import static java.lang.Math.abs;

//Disabled
@Autonomous(name = "BLUE Outer Half", group = "Auto", preselectTeleOp = "Wolfpack TeleOp")
public class BlueOuterHalf extends OpMode {
	
	private Controller operator;
	
	private Main currentMainState = Main.delay1;
	private final ElapsedTime mainTime = new ElapsedTime();
	private static double ringCount = 0;
	private final boolean ringsFound = false;

	// 0 RING DELAYS //
	private static final double START0 = 0; private static final double PRELOAD0 = 0; private static final double WOBBLE0 = 0; private static final double PARK0 = 0;

	// 1 RING DELAYS //
	private static final double START1 = 0; private static final double PRELOAD1 = 0; private static final double STACK1 = 0;private static final double WOBBLE1 = 0;



	@Override
	public void init() {
		Utils.setOpMode(this);

		Robot.init(); Sensors.init(); Shooter.init(); Intake.init(); Wobble.init();
		Sensors.alliance = Sensors.Alliance.BLUE;
		
		operator = new Controller(gamepad2);

	}
	
	public void init_loop() {
		operator.update();
		Shooter.resetFeeder(); Shooter.lockFeeder(); Shooter.shooterOff();

		Sensors.frontCamera.calibrateTowerDetection();

		if (operator.RB.inputYet()) {
			if (operator.RB.toggle()) Wobble.gripperGrip();
			else Wobble.gripperHalf();
		}

		if (operator.cross.inputYet()) {
			if (operator.cross.toggle()) { Intake.intakeStallControl(); Intake.bumperGroundRings(); }
			else { Intake.intakeOff(); Intake.bumperRetract(); }
		}
		
		telemetry.addData("Ring Count = ", Sensors.backCamera.startingStackCount());
		telemetry.update();
		
		sleep(40);
	}
	
	public void start() {
		Sensors.update();
		mainTime.reset(); Robot.resetGyro(90); Robot.resetWithoutEncoders();
		ringCount = 1;
		//ringCount = Sensors.backCamera.startingStackCount();
		Shooter.setFeederCount(0); Shooter.setTurretAngle(0);
		Intake.intakeOff(); Intake.bumperRetract();
	}
	
	@RequiresApi(api = Build.VERSION_CODES.N)
	@Override
	public void loop() {
		Sensors.update(); operator.update();
		switch ((int) ringCount){
			case 0:
				switch (currentMainState){
					case delay1:
						if(mainTime.seconds() >= START0) newState(Main.state1Drive);
						break;

					case state1Drive:
						Robot.strafe(22, -90, 90, 1, .15, 0);
						Wobble.armTele();
						if (Robot.isStrafeComplete) newState(Main.state2Turn);
						break;

					case state2Turn:
						if (mainTime.seconds() > .8) {
							Robot.turn(85, 1, .3);
							if (Sensors.gyro.angleRange(60, 100)) newState(Main.state3Shoot);
						}
						break;

					case state3Shoot:
						Robot.setPowerVision(0, 0, Sensors.gyro.rawAngle() + Sensors.frontCamera.highGoalError());
						Shooter.highGoal(true);
						Shooter.feederState(abs(Sensors.frontCamera.highGoalError()) < 3 && Shooter.getRPM() > (Shooter.targetRPM - 60) && Shooter.getRPM() < (Shooter.targetRPM + 60));
						if (Shooter.feederCount() >= 4) newState(Main.delay2);
						break;

					case delay2:
						Shooter.shooterOff(); Shooter.resetFeeder();
						if(mainTime.seconds() > PRELOAD0) newState(Main.state4Turn);
						break;

					case state4Turn:
						Wobble.armPosition(.15);
						if (mainTime.seconds() > .1) {
							Robot.turn(-70, 1, .2);
							if (Sensors.gyro.angleRange(-74, -65)) newState(Main.state5Drive);
							break;
						}

					case state5Drive:
						if(mainTime.seconds() > .3) Robot.strafe(4, -70, 110, .6, .2, 0);
						if(mainTime.seconds() > .4 && Robot.isStrafeComplete) newState(Main.delay3);
						break;

					case delay3:
						Robot.setPowerAuto(0, 0, -70);
						if(mainTime.seconds() > .3) Wobble.gripperOpen();
						if(mainTime.seconds() > .5) Wobble.armFold();
						if(mainTime.seconds() > .7) Wobble.gripperHalf();
						if(mainTime.seconds() > .7 + WOBBLE0) newState(Main.state6Drive);
						break;

					case state6Drive:
						Robot.strafe(17, -90, -90, 1, .3, 0);
						if(mainTime.seconds() > .1 && Robot.isStrafeComplete) newState(Main.state7Turn);
						break;

					case state7Turn:
						if(mainTime.seconds() >= .8) Robot.turn(72, 1, .2);
						if(mainTime.seconds() >= .9 && Sensors.gyro.angleRange(67, 77)) newState(Main.delay4);
						break;

					case delay4:
						if(mainTime.seconds() >= .1 + PARK0) newState(Main.state7Drive);
						break;

					case state7Drive:
						Robot.strafe(20, 72, 72, 1, .3, 0);
						if(mainTime.seconds() > .1 && Robot.isStrafeComplete) newState(Main.state8Turn);
						break;

					case state8Turn:
						if(mainTime.seconds() > .8) Robot.setPowerAuto(0, 0, 90);
						break;
				}

			case 1:
				switch (currentMainState) {
					case delay1:
						if (mainTime.seconds() >= START1) newState(Main.state1Drive);
						break;

					case state1Drive:
						Robot.strafe(22, -90, 90, 1, .15, 0);
						Wobble.armTele();
						if (Robot.isStrafeComplete) newState(Main.state2Turn);
						break;

					case state2Turn:
						if (mainTime.seconds() > .8) {
							Robot.turn(75, 1, .3);
							if (Sensors.gyro.angleRange(60, 100)) newState(Main.state3Shoot);
						}
						break;

					case state3Shoot:
						Robot.setPowerVision(0, 0, Sensors.gyro.rawAngle() + Sensors.frontCamera.highGoalError());
						Shooter.highGoal(true);
						Shooter.feederState(abs(Sensors.frontCamera.highGoalError()) < 3 && Shooter.getRPM() > (Shooter.targetRPM - 60) && Shooter.getRPM() < (Shooter.targetRPM + 60));
						if (Shooter.feederCount() >= 4) newState(Main.delay2);
						break;

					case delay2:
						Shooter.shooterOff();Shooter.resetFeeder();
						Wobble.armFold();
						if (mainTime.seconds() > PRELOAD1) newState(Main.state4Turn);
						break;

					case state4Turn:
						if (mainTime.seconds() > .1) {
							Robot.turn(-110, 1, .2);
							if (Sensors.gyro.angleRange(-115, -100)) newState(Main.state5Drive);
							break;
						}

					case state5Drive:
						Wobble.armPosition(.15);
						Robot.strafe(13, -110, 70, .7, .2, 0);
						if(mainTime.seconds() > .1 && Robot.isStrafeComplete) newState(Main.delay3);
						break;

					case delay3:
						if(mainTime.seconds() > .4) { Wobble.gripperOpen(); Robot.setPowerAuto(0, 0, Robot.closestTarget(-110)); }
						if(mainTime.seconds() > .5) Wobble.armFold();
						if(mainTime.seconds() > .8) Wobble.gripperHalf();
						if(mainTime.seconds() > .8 + WOBBLE1) newState(Main.state6Drive);
						break;

					case state6Drive:
						Robot.strafe(20, -100, -100, .7, .2, 0);
						if(mainTime.seconds() > .1 && Robot.isStrafeComplete) newState(Main.state7Turn);
						break;

					case state7Turn:
						Intake.bumperGroundRings();
						if(mainTime.seconds() >= .8) { Robot.turn(20, .7, .2); Intake.intakeOn(); }
						if(mainTime.seconds() >= .9 && Sensors.gyro.angleRange(-25, 20)) newState(Main.state8Turn);
						break;

					case state8Turn:
						Intake.bumperGroundRings();
						Robot.turn(90, .7, .7);
						Intake.intakeOn();
						if(mainTime.seconds() >= .1 && Sensors.gyro.angleRange(85, 95)) newState(Main.delay4);
						break;

					case delay4:
						if(mainTime.seconds() >= .1 + STACK1) newState(Main.state8Drive);
						if(mainTime.seconds() > .5) { Intake.intakeOff(); Intake.bumperRetract(); }
						break;

					case state8Drive:
						Robot.strafe(5, 90, 90, 1, .3, 0);
						Shooter.setFeederCount(0);
						if(Robot.currentInches > 4) { Intake.intakeOff(); Intake.bumperRetract(); }
						if(mainTime.seconds() > .1 && Robot.isStrafeComplete) newState(Main.state9Shoot);
						break;

					case state9Shoot:
						if(mainTime.seconds() > .5) {
							Robot.setPowerVision(0, 0, Sensors.gyro.rawAngle() + Sensors.frontCamera.highGoalError());
							Shooter.highGoal(true);
							Shooter.feederState(abs(Sensors.frontCamera.highGoalError()) < 3 && Shooter.getRPM() > (Shooter.targetRPM - 60) && Shooter.getRPM() < (Shooter.targetRPM + 60));
							if (Shooter.feederCount() >= 2) newState(Main.state10Turn);
						}
						break;

					case state10Turn:
						Shooter.shooterOff(); Shooter.resetFeeder(); Shooter.lockFeeder();
						Robot.turn(-90, 1, .2);
						if(Sensors.gyro.angleRange(-95, -85)) newState(Main.state11Drive);
						break;

					case state11Drive:
						Robot.strafe(5, -90, 90, .7, .2, 0);
						if(mainTime.seconds() > .1 && Robot.isStrafeComplete) newState(Main.stateFinished);
						break;

				}
		}
		
		loopTelemetry();
	}
	
	
	private void newState(Main newState) {
		currentMainState = newState;
		mainTime.reset();
	}
	
	private enum Main {
		state1Drive, state2Turn, state2WobbleGoal, state3Turn, state4Drive, state5Turn, state6PS1, state7PS2, state8PS3, state9Turn, state9Drive, state10WobbleGoal, state11Drive, state11Turn,
		state12WobbleGoal, state12Drive, state13Turn, state13Drive, state14Drive, state15Drive, state15Shoot, state16Shoot, state16Drive, state17Shoot, state17Drive, state18Turn, state18Drive,
		state18Shoot, state19Drive, state20Drive, state20WobbleGoal, state21Drive, stateFinished, state1Diagonal, state1Turn, state22Shoot, state22Turn, state3Shoot, state10Drive, state10Turn,
		state14Turn, state17Wobble, state15Turn, state16Turn, state19Wobble, state20Turn, state95Drive, state22Drive, state165Turn, state115Drive, delay1, delay2, state23Drive, state4Turn,
		state5Drive, state6Drive, delay4, state7Drive, state8Turn, state7Turn, delay5, state8Drive, state9Shoot, delay3
	}
	
	private void loopTelemetry(){
		telemetry.addData("current state: ", currentMainState);
		telemetry.addData("drive", Robot.drive);
		telemetry.addData("strafe", Robot.strafe);
		telemetry.addData("turn", Robot.turn);
		telemetry.addData("power", Robot.power);
		telemetry.addData("current angle", Sensors.gyro.modAngle());
		telemetry.update();
}
}


