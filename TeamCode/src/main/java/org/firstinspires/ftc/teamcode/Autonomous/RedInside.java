
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
import org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_Sanic;
import org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.PowerShot;
import org.firstinspires.ftc.teamcode.HardwareClasses.Sensors;
import org.firstinspires.ftc.teamcode.HardwareClasses.Shooter;
import org.firstinspires.ftc.teamcode.HardwareClasses.Wobble;
import org.firstinspires.ftc.teamcode.utilities.Utils;

import static android.os.SystemClock.sleep;
import static java.lang.Math.abs;
import static org.firstinspires.ftc.teamcode.HardwareClasses.Robot.closestTarget;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.Auto.BLUE_INNER;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.Auto.RED_INNER;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.Target.BLUE_GOAL;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.Target.RED_GOAL;
import static org.firstinspires.ftc.teamcode.utilities.Utils.multTelemetry;

//Disabled
@Autonomous(name = "RED Inside", group = "Auto", preselectTeleOp = "Wolfpack TeleOp")
public class RedInside extends OpMode {

    private Controller operator;

    private Main currentMainState = Main.delay1;
    private final ElapsedTime mainTime = new ElapsedTime();
    private static double ringCount = 0;
    private boolean wasCalibrated = false;

    // 0 RING DELAYS //
    private static final double START0 = 0; private static final double POWERSHOT0 = 0; private static final double BOUNCEBACK0 = 0; private static final double CORNER0 = 0; private static final double WOBBLE0 = 0;

    // 1 RING DELAYS //
    private static final double START1 = 0; private static final double POWERSHOT1 = 0; private static final double BOUNCEBACK1 = 0; private static final double CORNER1 = 0; private static final double WOBBLE1 = 0;

    // 4 RING DELAYS //
    private static final double START4 = 0; private static final double POWERSHOT4 = 0; private static final double BOUNCEBACK4 = 0; private static final double CORNER4 = 0; private static final double WOBBLE4 = 0;



    @Override
    public void init() {
        Utils.setOpMode(this);
        operator = new Controller(gamepad2);

        Robot.init(); Sensors.init(); Shooter.init(); Intake.init(); Wobble.init();

        // Set the alliance and the auto orientation
        Sensors.alliance = Sensors.Alliance.RED;
        Dash_Sanic.AUTO = RED_INNER;

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

        mainTime.reset(); Robot.resetGyro(-90); Robot.resetWithoutEncoders();
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
                switch (currentMainState) {
                    case delay1:
                        if (mainTime.seconds() >= START0) newState(Main.state1Drive);
                        break;

                    case state1Drive:
                        Robot.strafe(24, 97, -83, 1, .15, 0);
                        Wobble.armTele();
                        if (mainTime.seconds() > .1 && Robot.isStrafeComplete) newState(Main.state2Turn);
                        break;

                    case state2Turn:
                        if (mainTime.seconds() > .8) {
                            Robot.turn(-100, 1, .3);
                            if (Sensors.gyro.angleRange(-106, -94)) newState(Main.state3PS1);
                        }
                        Shooter.setFeederCount(0);
                        break;

                    case state3PS1:
                        Shooter.powerShot();
                        Robot.setPowerVision(0, 0, Sensors.frontCamera.getPowerShotAngle(PowerShot.PS_MID) - 2);
                        Shooter.feederState(mainTime.seconds() > 1 &&
                                Shooter.getRPM() > (Shooter.targetRPM - 50) && Shooter.getRPM() < (Shooter.targetRPM + 50));
                        if (mainTime.seconds() > .7 && Shooter.feederCount() > 0)  newState(Main.state4PS2);
                        break;

                    case state4PS2:
                        Shooter.powerShot();
                        Robot.setPowerVision(0, 0, Sensors.frontCamera.getPowerShotAngle(PowerShot.PS_FAR) - 2);
                        Shooter.feederState(mainTime.seconds() > .6 &&
                                Shooter.getRPM() > (Shooter.targetRPM - 50) && Shooter.getRPM() < (Shooter.targetRPM + 50));
                        if (mainTime.seconds() > .7 && Shooter.feederCount() > 1) newState(Main.state5PS3);
                        break;

                    case state5PS3:
                        Shooter.powerShot();
                        Robot.setPowerVision(0, 0, Sensors.frontCamera.getPowerShotAngle(PowerShot.PS_CLOSE) - 1);
                        Shooter.feederState(mainTime.seconds() > .6 &&
                                Shooter.getRPM() > (Shooter.targetRPM - 50) && Shooter.getRPM() < (Shooter.targetRPM + 50));
                        if (mainTime.seconds() > .7 && Shooter.feederCount() > 3) newState(Main.delay2);
                        break;

                    case delay2:
                        Shooter.shooterOff(); Shooter.lockFeeder(); Shooter. resetFeeder();
                        if(mainTime.seconds() > POWERSHOT0) newState(Main.state6Drive);
                        break;

                    case state6Drive:
                        Robot.strafe(27, -90, -90, 1, .2, 0);
                        Intake.intakeOn(); Intake.bumperGroundRings();
                        if (mainTime.seconds() > .2 && Robot.isStrafeComplete) newState(Main.delay3);
                        break;

                    case delay3:
                        if(mainTime.seconds() > BOUNCEBACK0) newState(Main.state7Turn);
                        break;

                    case state7Turn:
                        if (mainTime.seconds() > .8) {
                            Robot.turn(-60, 1, .4);
                            if (Sensors.gyro.angleRange(-65, -55)) newState(Main.state8Turn);
                        }
                        break;

                    case state8Turn:
                        Robot.turn(-180, 1, .2);
                        if (Sensors.gyro.angleRange(-185, -175)) newState(Main.state9Drive);
                        break;

                    case state9Drive:
                        Robot.strafe(9, -180, -90, .8, .3, 0);
                        if (mainTime.seconds() > .2 && (mainTime.seconds() > 1 || Robot.isStrafeComplete))
                            newState(Main.delay4);
                        break;

                    case delay4:
                        if(mainTime.seconds() > CORNER0) newState(Main.state10Drive);
                        break;

                    case state10Drive:
                        Robot.strafe(21, -180, -180, 1, .3, 0);
                        if (mainTime.seconds() > .2 && Robot.isStrafeComplete) newState(Main.state11Drive);
                        break;

                    case state11Drive:
                        if (mainTime.seconds() > .7) Robot.strafe(28, -180, 90, 1, .3, 0);
                        if(mainTime.seconds() > .8 && Robot.isStrafeComplete) newState(Main.state12Turn);
                        break;

                    case state12Turn:
                        if (mainTime.seconds() > .5) {
                            Intake.intakeOff();
                            Intake.bumperRetract();
                        }if(mainTime.seconds() > .8){
                            Robot.turn(-105, 1, .3);
                            Wobble.armPosition(.15);
                            if (Sensors.gyro.angleRange(-111, -99)) newState(Main.delay5);
                        }
                        break;

                    case delay5:
                        Robot.setPowerAuto(0, 0, closestTarget(-105));
                        if(mainTime.seconds() > .1) Wobble.gripperOpen();
                        if(mainTime.seconds() > .3) Wobble.armFold();
                        if(mainTime.seconds() > .5) Wobble.gripperHalf();
                        if(mainTime.seconds() > .6 + WOBBLE0) newState(Main.state13Drive);
                        break;

                    case state13Drive:
                        Robot.strafe(40, -100, -10, 1, .3, 0);
                        if(mainTime.seconds() > .1 && Robot.isStrafeComplete) newState(Main.state14Drive);
                        break;

                    case state14Drive:
                        Robot.strafe(15, -90, 90, 1, .3, 0);
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
                        Robot.strafe(7, -90, -90, .8, .2, 0);
                        Shooter.shooterOff(); Shooter.resetFeeder(); Shooter.lockFeeder();
                        if(mainTime.seconds() > .1 && Robot.isStrafeComplete) newState(Main.stateFinished);
                        break;

                    case stateFinished:
                        Robot.setPowerVision(0, 0, closestTarget(-90));
                        break;

                }
                break;


            case 1:
                switch (currentMainState) {
                    case delay1:
                        if (mainTime.seconds() >= START1) newState(Main.state1Drive);
                        break;

                    case state1Drive:
                        Robot.strafe(24, 97, -83, 1, .15, 0);
                        Wobble.armTele();
                        if (mainTime.seconds() > .1 && Robot.isStrafeComplete) newState(Main.state2Turn);
                        break;

                    case state2Turn:
                        if (mainTime.seconds() > .8) {
                            Robot.turn(-100, 1, .3);
                            if (Sensors.gyro.angleRange(-106, -94)) newState(Main.state3PS1);
                        }
                        Shooter.setFeederCount(0);
                        break;

                    case state3PS1:
                        Shooter.powerShot();
                        Robot.setPowerVision(0, 0, Sensors.frontCamera.getPowerShotAngle(PowerShot.PS_MID) - 2);
                        Shooter.feederState(mainTime.seconds() > 1 &&
                                Shooter.getRPM() > (Shooter.targetRPM - 50) && Shooter.getRPM() < (Shooter.targetRPM + 50));
                        if (mainTime.seconds() > .7 && Shooter.feederCount() > 0)  newState(Main.state4PS2);
                        break;

                    case state4PS2:
                        Shooter.powerShot();
                        Robot.setPowerVision(0, 0, Sensors.frontCamera.getPowerShotAngle(PowerShot.PS_FAR) - 2);
                        Shooter.feederState(mainTime.seconds() > .6 &&
                                Shooter.getRPM() > (Shooter.targetRPM - 50) && Shooter.getRPM() < (Shooter.targetRPM + 50));
                        if (mainTime.seconds() > .7 && Shooter.feederCount() > 1) newState(Main.state5PS3);
                        break;

                    case state5PS3:
                        Shooter.powerShot();
                        Robot.setPowerVision(0, 0, Sensors.frontCamera.getPowerShotAngle(PowerShot.PS_CLOSE) - 1);
                        Shooter.feederState(mainTime.seconds() > .6 &&
                                Shooter.getRPM() > (Shooter.targetRPM - 50) && Shooter.getRPM() < (Shooter.targetRPM + 50));
                        if (mainTime.seconds() > .7 && Shooter.feederCount() > 3) newState(Main.delay2);
                        break;

                    case delay2:
                        Shooter.shooterOff(); Shooter.lockFeeder(); Shooter. resetFeeder();
                        if(mainTime.seconds() > POWERSHOT1) newState(Main.state6Drive);
                        break;

                    case state6Drive:
                        Robot.strafe(27, -90, -90, 1, .2, 0);
                        Intake.intakeOn(); Intake.bumperGroundRings();
                        if (mainTime.seconds() > .2 && Robot.isStrafeComplete) newState(Main.delay3);
                        break;

                    case delay3:
                        if(mainTime.seconds() > BOUNCEBACK1) newState(Main.state7Turn);
                        break;

                    case state7Turn:
                        if (mainTime.seconds() > .8) {
                            Robot.turn(-60, 1, .4);
                            if (Sensors.gyro.angleRange(-65, -55)) newState(Main.state8Turn);
                        }
                        break;

                    case state8Turn:
                        Robot.turn(-180, 1, .2);
                        if (Sensors.gyro.angleRange(-185, -175)) newState(Main.state9Drive);
                        break;

                    case state9Drive:
                        Robot.strafe(9, -180, -90, .8, .3, 0);
                        if (mainTime.seconds() > .2 && (mainTime.seconds() > 1 || Robot.isStrafeComplete))
                            newState(Main.delay4);
                        break;

                    case delay4:
                        if(mainTime.seconds() > CORNER1) newState(Main.state10Drive);
                        break;

                    case state10Drive:
                        Robot.strafe(21, -180, -180, 1, .3, 0);
                        if (mainTime.seconds() > .2 && Robot.isStrafeComplete) newState(Main.state11Drive);
                        break;

                    case state11Drive:
                        if (mainTime.seconds() > .7) Robot.strafe(16, -170, 10, 1, .3, 0);
                        if (mainTime.seconds() > .8 && Robot.isStrafeComplete)
                            newState(Main.state12Turn);
                        if(mainTime.seconds() > .8 && Robot.currentInches > 10) { Intake.intakeOff(); Intake.bumperRetract(); }
                        break;

                    case state12Turn:
                        if(mainTime.seconds() > .8) {
                            Wobble.armPosition(.15);
                            Robot.turn(-60, 1, 1);
                        }
                        if (Sensors.gyro.angleRange(-65, -55)) newState(Main.state13Drive);
                        break;

                    case state13Drive:
                        Robot.strafe(4, -60, 120, .4, .2, 0);
                        if(Robot.isStrafeComplete && mainTime.seconds() > .1) newState(Main.delay5);
                        break;

                    case delay5:
                        if(mainTime.seconds() > .1) Wobble.gripperOpen();
                        if(mainTime.seconds() > .3) { Wobble.armFold(); Robot.strafe(7, -40, -40, .4, .2, 0); }
                        if(mainTime.seconds() > .5) Wobble.gripperHalf();
                        if(mainTime.seconds() > .6 + WOBBLE1 && Robot.isStrafeComplete) newState(Main.state14Turn);
                        break;

                    case state14Turn:
                        Robot.turn(-90, 1, .2);
                        if(Sensors.gyro.angleRange(-95, -85)) newState(Main.state15Drive);
                        break;

                    case state15Drive:
                        Robot.strafe(22, -90, 90, 1, .2, 0);
                        Shooter.setFeederCount(0);
                        if (mainTime.seconds() > .2 && Robot.isStrafeComplete) newState(Main.state16Shoot);
                        break;

                    case state16Shoot:
                        if(mainTime.seconds() > .7) {
                            Robot.setPowerVision(0, 0, Sensors.gyro.rawAngle() + Sensors.frontCamera.highGoalError());
                            Shooter.highGoal(true);
                            Shooter.feederState(abs(Sensors.frontCamera.highGoalError()) < 3 && Shooter.getRPM() > (Shooter.targetRPM - 60) && Shooter.getRPM() < (Shooter.targetRPM + 60));
                            if (Shooter.feederCount() >= 4) newState(Main.state17Drive);
                        }
                        break;

                    case state17Drive:
                        Robot.strafe(7, -90, -90, .8, .2, 0);
                        Shooter.shooterOff(); Shooter.resetFeeder(); Shooter.lockFeeder();
                        if(mainTime.seconds() > .1 && Robot.isStrafeComplete) newState(Main.stateFinished);
                        break;

                    case stateFinished:
                        Robot.setPowerVision(0, 0, closestTarget(-90));
                        break;

                }
                break;


            case 4:
                switch (currentMainState) {
                    case delay1:
                        if (mainTime.seconds() >= START4) newState(Main.state1Drive);
                        break;

                    case state1Drive:
                        Robot.strafe(24, 97, -83, 1, .15, 0);
                        Wobble.armTele();
                        if (mainTime.seconds() > .1 && Robot.isStrafeComplete) newState(Main.state2Turn);
                        break;

                    case state2Turn:
                        if (mainTime.seconds() > .8) {
                            Robot.turn(-100, 1, .3);
                            if (Sensors.gyro.angleRange(-106, -94)) newState(Main.state3PS1);
                        }
                        Shooter.setFeederCount(0);
                        break;

                    case state3PS1:
                        Shooter.powerShot();
                        Robot.setPowerVision(0, 0, Sensors.frontCamera.getPowerShotAngle(PowerShot.PS_MID) - 2);
                        Shooter.feederState(mainTime.seconds() > 1 &&
                                Shooter.getRPM() > (Shooter.targetRPM - 50) && Shooter.getRPM() < (Shooter.targetRPM + 50));
                        if (mainTime.seconds() > .7 && Shooter.feederCount() > 0)  newState(Main.state4PS2);
                        break;

                    case state4PS2:
                        Shooter.powerShot();
                        Robot.setPowerVision(0, 0, Sensors.frontCamera.getPowerShotAngle(PowerShot.PS_FAR) - 2);
                        Shooter.feederState(mainTime.seconds() > .6 &&
                                Shooter.getRPM() > (Shooter.targetRPM - 50) && Shooter.getRPM() < (Shooter.targetRPM + 50));
                        if (mainTime.seconds() > .7 && Shooter.feederCount() > 1) newState(Main.state5PS3);
                        break;

                    case state5PS3:
                        Shooter.powerShot();
                        Robot.setPowerVision(0, 0, Sensors.frontCamera.getPowerShotAngle(PowerShot.PS_CLOSE) - 1);
                        Shooter.feederState(mainTime.seconds() > .6 &&
                                Shooter.getRPM() > (Shooter.targetRPM - 50) && Shooter.getRPM() < (Shooter.targetRPM + 50));
                        if (mainTime.seconds() > .7 && Shooter.feederCount() > 3) newState(Main.delay2);
                        break;

                    case delay2:
                        Shooter.shooterOff(); Shooter.lockFeeder(); Shooter. resetFeeder();
                        if(mainTime.seconds() > POWERSHOT4) newState(Main.state6Drive);
                        break;

                    case state6Drive:
                        Robot.strafe(27, -90, -90, 1, .2, 0);
                        Intake.intakeOn(); Intake.bumperGroundRings();
                        if (mainTime.seconds() > .2 && Robot.isStrafeComplete) newState(Main.delay3);
                        break;

                    case delay3:
                        if(mainTime.seconds() > BOUNCEBACK4) newState(Main.state7Turn);
                        break;

                    case state7Turn:
                        if (mainTime.seconds() > .8) {
                            Robot.turn(-60, 1, .4);
                            if (Sensors.gyro.angleRange(-65, -55)) newState(Main.state8Turn);
                        }
                        break;

                    case state8Turn:
                        Robot.turn(-180, 1, .2);
                        if (Sensors.gyro.angleRange(-185, -175)) newState(Main.state9Drive);
                        break;

                    case state9Drive:
                        Robot.strafe(9, -180, -90, .8, .3, 0);
                        if (mainTime.seconds() > .2 && (mainTime.seconds() > 1 || Robot.isStrafeComplete))
                            newState(Main.delay4);
                        break;

                    case delay4:
                        if(mainTime.seconds() > CORNER4) newState(Main.state10Drive);
                        break;

                    case state10Drive:
                        Robot.strafe(21, -180, -180, 1, .3, 0);
                        if (mainTime.seconds() > .2 && Robot.isStrafeComplete) newState(Main.state11Drive);
                        break;

                    case state11Drive:
                        if (mainTime.seconds() > .7) {
                            Intake.intakeOff(); Intake.bumperRetract();
                            Robot.strafe(8, -160, 20, 1, .3, 0);
                        }
                        if (mainTime.seconds() > .8 && Robot.isStrafeComplete)
                            newState(Main.state12Turn);
                        break;

                    case state12Turn:
                        Robot.turn(-10, 1, .3);
                        if (Sensors.gyro.angleRange(-145, -130)) newState(Main.state13Turn);
                        break;

                    case state13Turn:
                        Wobble.armPosition(.15);
                        Robot.turn(10, 1, 1);
                        if (Sensors.gyro.angleRange(5, 15)) newState(Main.delay5);
                        break;

                    case delay5:
                        Robot.setPowerAuto(0, 0, closestTarget(10));
                        if(mainTime.seconds() > .1) Wobble.gripperOpen();
                        if(mainTime.seconds() > .3) Wobble.armFold();
                        if(mainTime.seconds() > .5) Wobble.gripperHalf();
                        if(mainTime.seconds() > .6 + WOBBLE4) newState(Main.state14Drive);
                        break;

                    case state14Drive:
                        Robot.strafe(11.5, 10, 10, 1, .3, 0);
                        if (mainTime.seconds() > .2 && Robot.isStrafeComplete)
                            newState(Main.state15Turn);
                        break;

                    case state15Turn:
                        if (mainTime.seconds() > .7) {
                            Robot.turn(-90, 1, .4);
                            if (Sensors.gyro.angleRange(-95, -85)) newState(Main.state16Drive);
                        }
                        break;

                    case state16Drive:
                        Robot.strafe(23, -90, 90, 1, .3, 0);
                        Shooter.setFeederCount(0);
                        if (mainTime.seconds() > .2 && Robot.isStrafeComplete) newState(Main.state17Shoot);
                        break;

                    case state17Shoot:
                        if(mainTime.seconds() > .7) {
                            Robot.setPowerVision(0, 0, Sensors.gyro.rawAngle() + Sensors.frontCamera.highGoalError());
                            Shooter.highGoal(true);
                            Shooter.feederState(abs(Sensors.frontCamera.highGoalError()) < 3 && Shooter.getRPM() > (Shooter.targetRPM - 60) && Shooter.getRPM() < (Shooter.targetRPM + 60));
                            if (Shooter.feederCount() >= 4) newState(Main.state18Drive);
                        }
                        break;

                    case state18Drive:
                        Robot.strafe(7, -90, -90, .8, .2, 0);
                        Shooter.shooterOff(); Shooter.resetFeeder(); Shooter.lockFeeder();
                        if(mainTime.seconds() > .1 && Robot.isStrafeComplete) newState(Main.stateFinished);
                        break;

                    case stateFinished:
                        Robot.setPowerVision(0, 0, closestTarget(-90));
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
        state5Drive, state6Drive, delay4, state7Drive, state8Turn, state7Turn, delay5, state8Drive, state9Shoot, stat36PS1, state3PS1, state4PS2, state5PS3, state12Turn, breakpoint, psDelay, breakpoint1, breakpoint2, breakpoint3, breakpoint4, delay3
    }

    private void loopTelemetry(){
        multTelemetry.addData("current state: ", currentMainState);
        multTelemetry.addData("drive", Robot.drive);
        multTelemetry.addData("strafe", Robot.strafe);
        multTelemetry.addData("turn", Robot.turn);
        multTelemetry.addData("power", Robot.power);
        multTelemetry.addData("ps angle", 0);
        multTelemetry.addData("current angle", Sensors.gyro.modAngle());
        multTelemetry.update();
    }
}


