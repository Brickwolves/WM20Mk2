
package org.firstinspires.ftc.teamcode.Autonomous;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareClasses.Controller;
import org.firstinspires.ftc.teamcode.HardwareClasses.Intake;
import org.firstinspires.ftc.teamcode.HardwareClasses.Robot;
import org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_Sanic;
import org.firstinspires.ftc.teamcode.HardwareClasses.Sensors;
import org.firstinspires.ftc.teamcode.HardwareClasses.Shooter;
import org.firstinspires.ftc.teamcode.HardwareClasses.Wobble;
import org.firstinspires.ftc.teamcode.utilities.Utils;

import static android.os.SystemClock.sleep;
import static java.lang.Math.abs;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.Auto.BLUE_OUTER;

//Disabled
@Autonomous(name = "BLUE Outside", group = "Auto", preselectTeleOp = "Wolfpack TeleOp")
public class BlueOutside extends OpMode {

	private Controller operator;

	private Main currentMainState = Main.delay1;
	private final ElapsedTime mainTime = new ElapsedTime();
	private static double ringCount = 0;
	private boolean wasCalibrated = false;

	// 0 RING DELAYS //
	private static final double START0 = 0; private static final double PRELOAD0 = 0; private static final double WOBBLE0 = 0; private static final double PARK0 = 0;

	// 1 RING DELAYS //
	private static final double START1 = 0; private static final double PRELOAD1 = 0; private static final double STACK1 = 0;private static final double WOBBLE1 = 0;

	// 4 RING DELAYS //
	private static final double START4 = 0; private static final double PRELOAD4 = 0; private static final double STACK4 = 0;private static final double WOBBLE4 = 0;

	private static final int stackRPMOffset = 70;


	@Override
	public void init() {
		Utils.setOpMode(this);
		operator = new Controller(gamepad2);

		Robot.init(); Sensors.init(); Shooter.init(); Intake.init(); Wobble.init();

		Sensors.alliance = Sensors.Alliance.BLUE;
		Dash_Sanic.AUTO = BLUE_OUTER;

		mainTime.reset();
	}
	
	public void init_loop() {
		operator.update();
		Shooter.shooterOff(); Intake.intakeOff();

		// Calibrate the tower
		Sensors.frontCamera.calibrateTowerDetection();
		wasCalibrated = mainTime.seconds() > 5;
		
		telemetry.addData("Ring Count = ", Sensors.backCamera.startingStackCount());
		telemetry.update();
		
		sleep(40);
	}
	
	public void start() {
		Sensors.update();
		Shooter.resetFeeder(); Shooter.lockFeeder();
		Wobble.gripperGrip();

		mainTime.reset(); Robot.resetGyro(90); Robot.resetWithoutEncoders();
		ringCount = Sensors.backCamera.startingStackCount();
		Shooter.setFeederCount(0); Shooter.setTurretAngle(0);
		Intake.intakeOff(); Intake.bumperRetract();
	}
	
	@RequiresApi(api = Build.VERSION_CODES.N)
	@Override
	public void loop() {
		Sensors.update();
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
							Robot.turn(-50, 1, .2);
							if (Sensors.gyro.angleRange(-54, -45)) newState(Main.state5Drive);
							break;
						}

					case state5Drive:
						if(mainTime.seconds() > .3) Robot.strafe(4, -50, 130, .6, .2, 0);
						if(mainTime.seconds() > .4 && Robot.isStrafeComplete) newState(Main.delay3);
						break;

					case delay3:
						Robot.setPowerAuto(0, 0, -50);
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
						if(mainTime.seconds() > .8) Robot.setPowerVision(0, 0, 90);
						break;
				}
				break;

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
						Robot.strafe(9.5, -110, 70, .7, .2, 0);
						if(mainTime.seconds() > .1 && Robot.isStrafeComplete) newState(Main.delay3);
						break;

					case delay3:
						if(mainTime.seconds() > .4) { Wobble.gripperOpen(); Robot.setPowerAuto(0, 0, Robot.closestTarget(-110)); }
						if(mainTime.seconds() > .5) Wobble.armFold();
						if(mainTime.seconds() > .8) Wobble.gripperHalf();
						if(mainTime.seconds() > .8 + WOBBLE1) newState(Main.state6Drive);
						break;

					case state6Drive:
						Robot.strafe(16, -100, -100, .7, .2, 0);
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
						Robot.strafe(7, 90, 90, 1, .3, 0);
						Shooter.setFeederCount(0);
						if(mainTime.seconds() > .1 && Robot.isStrafeComplete) newState(Main.state9Shoot);
						break;

					case state9Shoot:
						if(mainTime.seconds() > .5) {
							Intake.intakeOff(); Intake.bumperRetract();
							Robot.setPowerVision(0, 0, Sensors.gyro.rawAngle() + Sensors.frontCamera.highGoalError());
							Shooter.highGoal(true);
							Shooter.feederState(abs(Sensors.frontCamera.highGoalError()) < 3 && Shooter.getRPM() > (Shooter.targetRPM - 60) && Shooter.getRPM() < (Shooter.targetRPM + 60));
							if (Shooter.feederCount() >= 2) newState(Main.state10Drive);
						}
						break;

					case state10Drive:
						Shooter.shooterOff(); Shooter.resetFeeder(); Shooter.lockFeeder();
						Robot.strafe(4, 90, 90, .7, .2, 0);
						if(mainTime.seconds() > .1 && Robot.isStrafeComplete) newState(Main.stateFinished);
						break;

					case stateFinished:
						if(mainTime.seconds() > .7) Robot.setPowerVision(0, 0, Robot.closestTarget(90));
						break;

				}
				break;

			case 4:
				switch (currentMainState) {
					case delay1:
						if (mainTime.seconds() >= START4) newState(Main.state1Drive);
						break;

					case state1Drive:
						Robot.strafe(22, -90, 90, 1, .15, 0);
						Wobble.armTele();
						if (mainTime.seconds() > .2 && Robot.isStrafeComplete) newState(Main.state2Turn);
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
						Shooter.shooterOff();
						Shooter.resetFeeder();
						Wobble.armFold();
						if (mainTime.seconds() > PRELOAD4) newState(Main.state4Drive);
						break;

					case state4Drive:
						Robot.strafe(24.2, 90, -90, 1, .15, 0);
						Intake.bumperGroundRings();
						if (mainTime.seconds() > .2 && Robot.isStrafeComplete) newState(Main.state5Drive);
						break;

					case state5Drive:
						Robot.strafe(11, 90, 0, 1, .2, 0);
						if (mainTime.seconds() > .2 && Robot.isStrafeComplete) newState(Main.state6Drive);
						break;

					case state6Drive:
						Wobble.armTele();
						//Intake.intakeOn();
						Shooter.highGoal(true);
						Robot.strafe(3, 90, 90, .5, .2, 0);
						if (mainTime.seconds() > .2 && Robot.isStrafeComplete) newState(Main.state6Shoot);
						break;

					case state6Shoot:
						Wobble.armTele();
						Robot.strafe(3, Sensors.gyro.rawAngle() + Sensors.frontCamera.highGoalError(), Sensors.gyro.rawAngle() + Sensors.frontCamera.highGoalError(), .12, .12, 0);
						Shooter.turretAim(); Shooter.highGoal(true, stackRPMOffset);
						Intake.intakeOn();
						Shooter.feederState(abs(Sensors.frontCamera.highGoalError()) < 3 && Shooter.getRPM() > (Shooter.targetRPM - 60) && Shooter.getRPM() < (Shooter.targetRPM + 60));
						if (mainTime.seconds() > .2 && Robot.isStrafeComplete) newState(Main.state7Drive);
						break;

					case state7Drive:
						Robot.strafe(.5, Sensors.gyro.rawAngle() + Sensors.frontCamera.highGoalError(), -90, .09, .0, 0);
						Shooter.turretAim(); Shooter.highGoal(true, stackRPMOffset);
						if (mainTime.seconds() > .2 && Robot.isStrafeComplete) newState(Main.state8Shoot);
						break;

					case state8Shoot:
						Robot.strafe(3, Sensors.gyro.rawAngle() + Sensors.frontCamera.highGoalError(), Sensors.gyro.rawAngle() + Sensors.frontCamera.highGoalError(), .12, .12, 0);
						Shooter.turretAim(); Shooter.highGoal(true, stackRPMOffset);
						Shooter.feederState(abs(Sensors.frontCamera.highGoalError()) < 3 && Shooter.getRPM() > (Shooter.targetRPM - 60) && Shooter.getRPM() < (Shooter.targetRPM + 60));
						if (mainTime.seconds() > .2 && Robot.isStrafeComplete) newState(Main.state9Drive);
						break;

					case state9Drive:
						Robot.strafe(.5, Sensors.gyro.rawAngle() + Sensors.frontCamera.highGoalError(), -90, .09, .0, 0);
						Shooter.turretAim(); Shooter.highGoal(true, stackRPMOffset);
						if (mainTime.seconds() > .2 && Robot.isStrafeComplete) newState(Main.state10Shoot);
						break;

					case state10Shoot:
						Robot.strafe(3, Sensors.gyro.rawAngle() + Sensors.frontCamera.highGoalError(), Sensors.gyro.rawAngle() + Sensors.frontCamera.highGoalError(), .12, .12, 0);
						Shooter.turretAim(); Shooter.highGoal(true, stackRPMOffset);
						Shooter.feederState(abs(Sensors.frontCamera.highGoalError()) < 3 && Shooter.getRPM() > (Shooter.targetRPM - 60) && Shooter.getRPM() < (Shooter.targetRPM + 60));
						if (mainTime.seconds() > .2 && Robot.isStrafeComplete) newState(Main.state11Drive);
						break;

					case state11Drive:
						Robot.strafe(.5, Sensors.gyro.rawAngle() + Sensors.frontCamera.highGoalError(), -90, .09, .0, 0);
						Shooter.turretAim(); Shooter.highGoal(true, stackRPMOffset);
						if (mainTime.seconds() > .2 && Robot.isStrafeComplete) newState(Main.state12Shoot);
						break;

					case state12Shoot:
						Robot.strafe(3, Sensors.gyro.rawAngle() + Sensors.frontCamera.highGoalError(), Sensors.gyro.rawAngle() + Sensors.frontCamera.highGoalError(), .12, .12, 0);
						Shooter.turretAim(); Shooter.highGoal(true, stackRPMOffset);
						Shooter.feederState(abs(Sensors.frontCamera.highGoalError()) < 3 && Shooter.getRPM() > (Shooter.targetRPM - 60) && Shooter.getRPM() < (Shooter.targetRPM + 60));
						if (mainTime.seconds() > .2 && Robot.isStrafeComplete) newState(Main.state13Drive);
						break;

					case state13Drive:
						Robot.strafe(.5, Sensors.gyro.rawAngle() + Sensors.frontCamera.highGoalError(), -90, .09, .0, 0);
						Shooter.turretAim(); Shooter.highGoal(true, stackRPMOffset);
						if (mainTime.seconds() > .2 && Robot.isStrafeComplete) newState(Main.state14Shoot);
						break;

					case state14Shoot:
						Robot.strafe(3, Sensors.gyro.rawAngle() + Sensors.frontCamera.highGoalError(), Sensors.gyro.rawAngle() + Sensors.frontCamera.highGoalError(), .12, .12, 0);
						Shooter.turretAim(); Shooter.highGoal(true, stackRPMOffset);
						Shooter.feederState(abs(Sensors.frontCamera.highGoalError()) < 3 && Shooter.getRPM() > (Shooter.targetRPM - 60) && Shooter.getRPM() < (Shooter.targetRPM + 60));
						if (mainTime.seconds() > .2 && Robot.isStrafeComplete) newState(Main.state15Drive);
						break;

					case state15Drive:
						Robot.strafe(.5, Sensors.gyro.rawAngle() + Sensors.frontCamera.highGoalError(), -90, .09, .0, 0);
						Shooter.turretAim(); Shooter.highGoal(true, stackRPMOffset);
						if (mainTime.seconds() > .2 && Robot.isStrafeComplete) newState(Main.state16Shoot);
						break;

					case state16Shoot:
						Robot.strafe(3, Sensors.gyro.rawAngle() + Sensors.frontCamera.highGoalError(), Sensors.gyro.rawAngle() + Sensors.frontCamera.highGoalError(), .12, .12, 0);
						Shooter.turretAim(); Shooter.highGoal(true, stackRPMOffset);
						Shooter.feederState(abs(Sensors.frontCamera.highGoalError()) < 3 && Shooter.getRPM() > (Shooter.targetRPM - 60) && Shooter.getRPM() < (Shooter.targetRPM + 60));
						if (mainTime.seconds() > .2 && Robot.isStrafeComplete) newState(Main.delay3);
						break;


					case delay3:
						if(mainTime.seconds() > STACK4) newState(Main.state11Turn);
						break;

					case state11Turn:
						Shooter.shooterOff(); Shooter.setTurretAngle(0);
						Intake.intakeOff(); Intake.bumperRetract();
						Robot.turn(-73,1, .3);
						if(Sensors.gyro.angleRange(-78, -68)) newState(Main.state12Drive);
						break;

					case state12Drive:
						Robot.strafe(30, -73, 107, 1, .3, 0);
						if(Robot.currentInches > 19) Wobble.armPosition(.15);
						if(mainTime.seconds() > .2 && Robot.isStrafeComplete) newState(Main.delay4);
						break;

					case delay4:
						if(mainTime.seconds() > .8) { Wobble.gripperOpen(); Robot.setPowerAuto(0, 0, Robot.closestTarget(-73)); }
						if(mainTime.seconds() > 1) Wobble.armFold();
						if(mainTime.seconds() > 1.2) Wobble.gripperHalf();
						if(mainTime.seconds() > 1.2 + WOBBLE4) newState(Main.state17Drive);
						break;

					case state17Drive:
						Robot.strafe(14, -90, -90, 1, .2, 0);
						if(mainTime.seconds() > .1 && Robot.isStrafeComplete) newState(Main.state14Turn);
						break;

					case state14Turn:
						if(mainTime.seconds() > .8) Robot.turn(90, 1, .3);
						if(Sensors.gyro.angleRange(85, 95)) newState(Main.stateFinished);
						break;

					case stateFinished:
						Robot.setPowerVision(0, 0, Robot.closestTarget(90));
						break;
				}
				break;
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
		state5Drive, state6Drive, delay4, state7Drive, state8Turn, state7Turn, delay5, state8Drive, state9Shoot, state6Shoot, state8Shoot, state10Shoot, state12Shoot, state14Shoot, delay3
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


