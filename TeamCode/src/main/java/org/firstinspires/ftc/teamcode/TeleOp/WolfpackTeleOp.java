package org.firstinspires.ftc.teamcode.TeleOp;import android.os.Build;import androidx.annotation.RequiresApi;import com.qualcomm.robotcore.eventloop.opmode.OpMode;import com.qualcomm.robotcore.eventloop.opmode.TeleOp;import com.qualcomm.robotcore.util.ElapsedTime;import org.firstinspires.ftc.teamcode.HardwareClasses.Controller;import org.firstinspires.ftc.teamcode.HardwareClasses.Intake;import org.firstinspires.ftc.teamcode.HardwareClasses.Robot;import org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot;import org.firstinspires.ftc.teamcode.HardwareClasses.Sensors;import org.firstinspires.ftc.teamcode.HardwareClasses.Sensors.Alliance;import org.firstinspires.ftc.teamcode.HardwareClasses.Shooter;import org.firstinspires.ftc.teamcode.HardwareClasses.Wobble;import org.firstinspires.ftc.teamcode.utilities.Utils;import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.curTarget;import static org.firstinspires.ftc.teamcode.utilities.Utils.multTelemetry;//import org.firstinspires.ftc.teamcode.HardwareClasses.Shooter;@TeleOp(name = "Wolfpack TeleOp", group = "TeleOp")public class WolfpackTeleOp extends OpMode {		private boolean realMatch = true, autoAim = true;	private int gyroOffset = 0;	private String offsetTelemetry = "forward";		private final ElapsedTime mainTime = new ElapsedTime();	private Controller driver, operator;	private Alliance autoAlliance;				@Override	public void init() {		Utils.setOpMode(this);		autoAlliance = Sensors.alliance;		Robot.init(); Sensors.init(); Intake.init(); Shooter.init(); Wobble.init();				driver = new Controller(gamepad1); operator = new Controller(gamepad2);		mainTime.reset();	}				@RequiresApi(api = Build.VERSION_CODES.N)	@Override	public void init_loop() {		Sensors.update();		driver.update(); operator.update();		Sensors.frontCamera.calibrateTowerDetection();		if(autoAlliance == Alliance.BLUE) Sensors.alliance = driver.cross.toggle() ? Alliance.RED : Alliance.BLUE;		else Sensors.alliance = driver.cross.toggle() ? Alliance.BLUE : Alliance.RED;				Robot.setPower(0,0, driver.leftStick.X(), driver.RT.range(.3, .6));		Shooter.lockFeeder(); Shooter.resetFeeder(); Shooter.shooterOff(); Shooter.setTurretAngle(0);		Intake.intakeOff();				if(driver.up.press()) { gyroOffset = 0; offsetTelemetry = "forward"; }		if(driver.left.press()) { gyroOffset = -90; offsetTelemetry = "left"; }		if(driver.right.press()) { gyroOffset = 90; offsetTelemetry = "right"; }		if(driver.down.press()) { gyroOffset = 180; offsetTelemetry = "backward"; }		realMatch = !driver.share.toggle(); autoAim = !driver.triangle.toggle();				initTelemetry();	}				@Override	public void start() {		Sensors.update();		Robot.resetGyro(gyroOffset);		Wobble.newState(Wobble.GripperState.HALF); Wobble.newState(Wobble.ArmState.FOLD);		mainTime.reset();	}				@RequiresApi(api = Build.VERSION_CODES.N)	@Override	public void loop() {		Sensors.update(); driver.update(); operator.update();		driver.rightStick.setShift(Sensors.gyro.modAngle());		autoAim = !driver.triangle.toggle(); realMatch = !driver.share.toggle();		if(driver.square.press()){ Robot.resetGyro(-90); }				//movement controls		Robot.driveState(driver.rightStick.shiftedY(), driver.rightStick.shiftedX(), driver.leftStick.X(), driver.RT.range(.5, 1));		Robot.cardinalState(driver.up.press(), driver.right.press(), driver.down.press(), driver.left.press());		Robot.autoAimState(driver.RS.press(), autoAim, driver.circle.press(), driver.cross.press(), 9);				//operator controls		Shooter.shooterState(operator.triangle.press(), operator.left.press(), operator.right.press(), autoAim);		Shooter.feederState(operator.square.hold());				Intake.intakeState(operator.cross.press(), operator.circle.hold(), driver.LB.hold());		Intake.bumperState(operator.LT.press(), driver.LT.hold());				Wobble.armState(operator.LB.press(), operator.LS.press());		Wobble.gripperState(operator.RB.press());				//match timers		/*if(mainTime.seconds() > 87 && mainTime.seconds() < 87.5 && realMatch) Wobble.newState(Wobble.ArmState.UP);		if(mainTime.seconds() > 87.5 && mainTime.seconds() < 88 && realMatch) Wobble.newState(Wobble.ArmState.FOLD);		if((realMatch && mainTime.seconds() > 121) || driver.touchpad.hold()) requestOpModeStop();*/				loopTelemetry();	}					//TELEMETRY		private void initTelemetry(){		multTelemetry.addData("ALLIANCE", Sensors.alliance);		multTelemetry.addLine();		multTelemetry.addData("auto aim", autoAim);		multTelemetry.addData("real match", realMatch);		multTelemetry.addData("gyro offset", gyroOffset);		multTelemetry.addData("start angle", offsetTelemetry);		multTelemetry.update();	}		private void loopTelemetry(){		multTelemetry.addData("time", mainTime.seconds());		multTelemetry.addData("auto aim", autoAim);		multTelemetry.addData("real match", realMatch);		multTelemetry.addLine();				multTelemetry.addLine("// DRIVE TELEMETRY //");		multTelemetry.addData("MEASURED angle", Sensors.gyro.modAngle());		multTelemetry.addData("target angle", Robot.targetAngle);		multTelemetry.addData("error", Robot.targetAngle - Sensors.gyro.modAngle());		//multTelemetry.addData("robtt velocity component", Sensors.robotVelocityComponent(Sensors.frontCamera.highGoalError() - 90));		multTelemetry.addLine();				multTelemetry.addLine("// SHOOTER TELEMETRY //");		multTelemetry.addData("target rpm", Shooter.targetRPM);		//multTelemetry.addData("measured rpm", Shooter.getRPM());		multTelemetry.addData("turret angle", Shooter.getTurretAngle());		multTelemetry.addData("vertical component", Shooter.verticalComponent());		multTelemetry.addLine();				multTelemetry.addLine("// VISION TELEMETRY //");		multTelemetry.addData("BLUE_MAX_HSV", Dash_AimBot.BLUE_MAX_THRESH);		multTelemetry.addData("BLUE_MIN_HSV", Dash_AimBot.BLUE_MIN_THRESH);		multTelemetry.addData("RED_MAX_HSV", Dash_AimBot.RED_MAX_THRESH);		multTelemetry.addData("RED_MIN_HSV", Dash_AimBot.RED_MIN_THRESH);		multTelemetry.addData("Current Target", curTarget);		multTelemetry.addData("goal aim error", Sensors.frontCamera.aimBotPipe.getGoalDegreeError());		multTelemetry.addData("distance to goal", Sensors.frontCamera.aimBotPipe.getDistance2Goal());		multTelemetry.addLine();				multTelemetry.addLine("// INTAKE TELEMETRY //");		multTelemetry.addData("target rpm", Intake.targetRPM);		multTelemetry.addData("measured rpm", Intake.getRPM());		multTelemetry.addData("power", Intake.getPower());		multTelemetry.update();	}}