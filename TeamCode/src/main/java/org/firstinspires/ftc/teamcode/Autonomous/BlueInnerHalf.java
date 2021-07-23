
package org.firstinspires.ftc.teamcode.Autonomous;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareClasses.Controller;
import org.firstinspires.ftc.teamcode.HardwareClasses.Intake;
import org.firstinspires.ftc.teamcode.HardwareClasses.Robot;
import org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot;
import org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.PowerShot;
import org.firstinspires.ftc.teamcode.HardwareClasses.Sensors;
import org.firstinspires.ftc.teamcode.HardwareClasses.Shooter;
import org.firstinspires.ftc.teamcode.HardwareClasses.Wobble;
import org.firstinspires.ftc.teamcode.utilities.Utils;

import static android.os.SystemClock.sleep;
import static java.lang.Math.abs;
import static org.firstinspires.ftc.teamcode.HardwareClasses.Robot.closestTarget;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.Target.BLUE_GOAL;

//Disabled
@Autonomous(name = "BLUE Inner Half", group = "Auto", preselectTeleOp = "Wolfpack TeleOp")
public class BlueInnerHalf extends OpMode {
	
	private Controller operator;
	
	private Main currentMainState = Main.delay1;
	private final ElapsedTime mainTime = new ElapsedTime();
	private static double ringCount = 0;
	private final boolean ringsFound = false;

	// 0 RING DELAYS //
	private static final double START0 = 0; private static final double POWERSHOT0 = 0; private static final double BOUNCEBACK0 = 0; private static final double CORNER0 = 0; private static final double WOBBLE0 = 0;

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
		ringCount = 0;
		Dash_AimBot.curTarget = BLUE_GOAL;
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
				switch (currentMainState) {
					case delay1:
						if (mainTime.seconds() >= START0) newState(Main.state1Drive);
						break;

					case state1Drive:
						Robot.strafe(22, -97, 83, 1, .15, 0);
						Wobble.armTele();
						if (mainTime.seconds() > .1 && Robot.isStrafeComplete) newState(Main.state2Turn);
						break;

					case state2Turn:
						if (mainTime.seconds() > .8) {
							Robot.turn(100, .6, .3);
							if (Sensors.gyro.angleRange(94, 106)) newState(Main.state3PS1);
						}
						Shooter.setFeederCount(0);
						break;

					case breakpoint:
						telemetry.addData("PS aNGLE", Sensors.frontCamera.getPowerShotAngle(PowerShot.PS_MID));
						telemetry.addData("tower distance", Sensors.frontCamera.highGoalDistance());
						if(operator.cross.press()) newState(Main.state3PS1);
						break;

					case state3PS1:
						Shooter.powerShot();
						telemetry.addData("PS aNGLE", Sensors.frontCamera.getPowerShotAngle(PowerShot.PS_MID));
						telemetry.addData("tower distance", Sensors.frontCamera.highGoalDistance());
						Robot.setPowerVision(0, 0, Sensors.frontCamera.getPowerShotAngle(PowerShot.PS_MID));
						Shooter.feederState(mainTime.seconds() > .6 &&
								Shooter.getRPM() > (Shooter.targetRPM - 50) && Shooter.getRPM() < (Shooter.targetRPM + 50));
						if (mainTime.seconds() > .7 && Shooter.feederCount() > 0)  newState(Main.state4PS2);
						break;

					case state4PS2:
						Shooter.powerShot();
						Robot.setPowerVision(0, 0, Sensors.frontCamera.getPowerShotAngle(PowerShot.PS_FAR));
						Shooter.feederState(mainTime.seconds() > .6 &&
								Shooter.getRPM() > (Shooter.targetRPM - 50) && Shooter.getRPM() < (Shooter.targetRPM + 50));
						if (mainTime.seconds() > .7 && Shooter.feederCount() > 1) newState(Main.state5PS3);
						break;

					case state5PS3:
						Shooter.powerShot();
						Robot.setPowerVision(0, 0, Sensors.frontCamera.getPowerShotAngle(PowerShot.PS_CLOSE));
						Shooter.feederState(mainTime.seconds() > .6 &&
								Shooter.getRPM() > (Shooter.targetRPM - 50) && Shooter.getRPM() < (Shooter.targetRPM + 50));
						if (mainTime.seconds() > .7 && Shooter.feederCount() > 3) newState(Main.delay2);
						break;

					case delay2:
						Shooter.shooterOff(); Shooter.lockFeeder(); Shooter. resetFeeder(); Wobble.armFold();
						if(mainTime.seconds() > POWERSHOT0) newState(Main.state6Drive);
						break;

					case state6Drive:
						Robot.strafe(30, 88, 88, 1, .2, 0);
						Intake.intakeOn(); Intake.bumperGroundRings();
						if (mainTime.seconds() > .2 && Robot.isStrafeComplete) newState(Main.delay3);
						break;

					case delay3:
						if(mainTime.seconds() > BOUNCEBACK0) newState(Main.state8Turn);
						break;


					case state8Turn:
						if (mainTime.seconds() > .8) {
							Robot.turn(180, 1, .2);
							if (Sensors.gyro.angleRange(175, 185)) newState(Main.state9Drive);
						}
						break;

					case state9Drive:
						Robot.strafe(9, 180, 90, .8, .3, 0);
						if (mainTime.seconds() > .2 && (mainTime.seconds() > 1 || Robot.isStrafeComplete))
							newState(Main.delay4);
						break;

					case delay4:
						if(mainTime.seconds() > CORNER0) newState(Main.state10Drive);
						break;

					case state10Drive:
						Robot.strafe(24, 180, 180, 1, .3, 0);
						if (mainTime.seconds() > .2 && Robot.isStrafeComplete)
							newState(Main.state11Drive);
						break;

					case state11Drive:
						if (mainTime.seconds() > .7) Robot.strafe(28, 180, -90, 1, .3, 0);
						if(mainTime.seconds() > .8 && Robot.isStrafeComplete) newState(Main.state12Turn);
						break;

					case state12Turn:
						if (mainTime.seconds() > .5) {
							Intake.intakeOff(); Intake.bumperRetract();
							Robot.turn(70, 1, .3);
							Wobble.armPosition(.15);
							if (Sensors.gyro.angleRange(64, 76)) newState(Main.delay5);
						}
						break;

					case delay5:
						Robot.setPowerAuto(0, 0, closestTarget(70));
						if(mainTime.seconds() > .1) Wobble.gripperOpen();
						if(mainTime.seconds() > .3) Wobble.armFold();
						if(mainTime.seconds() > .5) Wobble.gripperHalf();
						if(mainTime.seconds() > .6 + WOBBLE0) newState(Main.state13Drive);
						break;

					case state13Drive:
						Robot.strafe(32, 100, 10, 1, .3, 0);
						if(mainTime.seconds() > .1 && Robot.isStrafeComplete) newState(Main.state14Drive);
						break;

					case state14Drive:
						Robot.strafe(16, 100, -80, 1, .3, 0);
						if(mainTime.seconds() > .1 && Robot.isStrafeComplete) newState(Main.state15Shoot);
						Shooter.setFeederCount(0);
						break;

					case state15Shoot:
						if(mainTime.seconds() > .7) {
							Robot.setPowerVision(0, 0, Sensors.gyro.rawAngle() + Sensors.frontCamera.highGoalError());
							Shooter.highGoal(true);
							Shooter.feederState(abs(Sensors.frontCamera.highGoalError()) < 3 && Shooter.getRPM() > (Shooter.targetRPM - 60) && Shooter.getRPM() < (Shooter.targetRPM + 60));
							if (Shooter.feederCount() >= 4) newState(Main.state16Drive);
						}
						break;

					case state16Drive:
						Robot.strafe(7, 90, 90, .8, .2, 0);
						Shooter.shooterOff(); Shooter.resetFeeder(); Shooter.lockFeeder();
						if(mainTime.seconds() > .1 && Robot.isStrafeComplete) newState(Main.stateFinished);
						break;

					case stateFinished:
						Robot.setPowerAuto(0, 0, closestTarget(90));
						break;

				}
				break;
		}
		
		loopTelemetry();
	}
	
	
	private void newState(Main newState) {
		if(mainTime.seconds() > .05) {
			currentMainState = newState;
			mainTime.reset();
		}
	}
	
	private enum Main {
		state1Drive, state2Turn, state2WobbleGoal, state3Turn, state4Drive, state5Turn, state6PS1, state7PS2, state8PS3, state9Turn, state9Drive, state10WobbleGoal, state11Drive, state11Turn,
		state12WobbleGoal, state12Drive, state13Turn, state13Drive, state14Drive, state15Drive, state15Shoot, state16Shoot, state16Drive, state17Shoot, state17Drive, state18Turn, state18Drive,
		state18Shoot, state19Drive, state20Drive, state20WobbleGoal, state21Drive, stateFinished, state1Diagonal, state1Turn, state22Shoot, state22Turn, state3Shoot, state10Drive, state10Turn,
		state14Turn, state17Wobble, state15Turn, state16Turn, state19Wobble, state20Turn, state95Drive, state22Drive, state165Turn, state115Drive, delay1, delay2, state23Drive, state4Turn,
		state5Drive, state6Drive, delay4, state7Drive, state8Turn, state7Turn, delay5, state8Drive, state9Shoot, stat36PS1, state3PS1, state4PS2, state5PS3, state12Turn, breakpoint, delay3
	}
	
	private void loopTelemetry(){
		telemetry.addData("current state: ", currentMainState);
		telemetry.addData("drive", Robot.drive);
		telemetry.addData("strafe", Robot.strafe);
		telemetry.addData("turn", Robot.turn);
		telemetry.addData("power", Robot.power);
		telemetry.addData("ps angle", 0);
		telemetry.addData("current angle", Sensors.gyro.modAngle());
		telemetry.update();
}
}


