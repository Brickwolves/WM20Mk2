package org.firstinspires.ftc.teamcode.HardwareClasses;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils;
import org.firstinspires.ftc.teamcode.utilities.MathUtils;
import org.firstinspires.ftc.teamcode.utilities.PID;
import org.firstinspires.ftc.teamcode.utilities.PID_Constants;
import org.firstinspires.ftc.teamcode.utilities.RingBuffer;
import org.firstinspires.ftc.teamcode.utilities.RingBufferOwen;

import static org.firstinspires.ftc.teamcode.utilities.MathUtils.pow;
import static org.firstinspires.ftc.teamcode.utilities.Utils.hardwareMap;
import static org.firstinspires.ftc.teamcode.utilities.Utils.multTelemetry;
import static org.firstinspires.ftc.teamcode.utilities.Velocity_Equation_Constants.*;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.curTarget;


public class Shooter {
    
    private static DcMotor shooterFront, shooterBack;
    private static Servo feeder, turret, feederLock;
    
    public static PID highGoalPID = new PID(0.00023, 0.00003, -0.000047, 0.3, 100, true);
    public static PID midGoalPID = new PID(.00029, 0.00003, -0.000047, 0.3, 100, false);
    public static PID powerShotPID = new PID(.00029, 0.00003, -0.000047, 0.3, 100,false);

    private static final double TICKS_PER_ROTATION = 42;
    
    private static final double RING_FEED = 0.15, RING_FULL_FEED = 0, HALF_RESET = 0.45, RESET = .63;
    private static final double FEEDER_LOCK = .46, FEEDER_UNLOCK = .18;
    
    private static final double FEED_TIME = .13, RESET_TIME = .1, PS_DELAY = .4;
    private static final double LOCK_TIME = 1, UNLOCK_TIME = .13;
    
    private static final double TURRET_SERVO_R = .935, TURRET_SERVO_L = .42, TURRET_SERVO_RANGE = TURRET_SERVO_R - TURRET_SERVO_L;
    private static final double TURRET_ANGLE_R = -22.5, TURRET_ANGLE_L = 39, TURRET_ANGLE_RANGE = TURRET_ANGLE_R - TURRET_ANGLE_L;
    
    private static final int TOP_GOAL = 3200, POWER_SHOT = 2800, MID_GOAL = 2900;
    
    private static boolean isFeederLocked;
    private static double shooterRPM, integralSumHigh, integralSumMid;
    private static int feedCount = 0;
    public static boolean shooterJustOn = false, feederJustOn = false;
    public static double targetRPM;

    static RingBufferOwen timeRing = new RingBufferOwen(2);
    static RingBufferOwen positionRing = new RingBufferOwen(2);
    static RingBuffer<Double> shortAngleRing = new RingBuffer<>(2,0.0);
    static RingBuffer<Long> shortTimeRing = new RingBuffer<>(2, (long)0);
    public static FeederState currentFeederState;
    public static ShooterState currentShooterState;
    
    public static ElapsedTime feederTime = new ElapsedTime();
    public static ElapsedTime shooterTime = new ElapsedTime();
    
    
    
    public static void init() {
        shooterFront = hardwareMap.get(DcMotor.class, "shooterfront");
        shooterBack = hardwareMap.get(DcMotor.class, "shooterback");
        feeder = hardwareMap.get(Servo.class, "feeder");
        feederLock = hardwareMap.get(Servo.class, "feederlock");
        turret = hardwareMap.get(Servo.class, "turret");
    
        
        shooterFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
    
        currentFeederState = FeederState.IDLE;
        currentShooterState = ShooterState.OFF;
    }
    
    
    public static void setTurretAngle(double turretAngle){
        
        long deltaMiliShort = Sensors.currentTimeMillis() - shortTimeRing.getValue(Sensors.currentTimeMillis());
        double deltaSecondsShort = deltaMiliShort / 1000.0;
    
        double deltaAngleShort = turretAngle - shortAngleRing.getValue(turretAngle);
        double rateOfChangeShort = deltaAngleShort/deltaSecondsShort;
        
        turretAngle += rateOfChangeShort * .17;
        
        turretAngle = Range.clip(turretAngle, TURRET_ANGLE_R, TURRET_ANGLE_L);
        
        double servoPos = (((turretAngle - TURRET_ANGLE_R) * TURRET_SERVO_RANGE) / TURRET_ANGLE_RANGE) + TURRET_SERVO_R;
        
        turret.setPosition(servoPos);
    }
    
    public static double getTurretAngle(){
        return (((turret.getPosition() - TURRET_SERVO_R) * TURRET_ANGLE_RANGE) / TURRET_SERVO_RANGE) + TURRET_ANGLE_R;
    }
    
    public static double verticalComponent(){
        double xComponent = MathUtils.degSin(getTurretAngle());
        double yComponent = Math.sqrt(.25 - .25 * Math.pow(xComponent, 2));
        return ((MathUtils.degASin(yComponent) - 30) * 1.1) + 30;
    }
    
    
    public static void turretAim(){ turretAim(true); }
    
    public static void turretAim(boolean autoAim){
        double towerError;
        towerError = (currentShooterState == ShooterState.MID_GOAL) ? Sensors.frontCamera.midGoalError() : Sensors.frontCamera.highGoalError();
        
        if(autoAim && Sensors.gyro.absAngleRange(67.5, 127.5) && getPower() > -.1)
            setTurretAngle(towerError - 0 + (Sensors.robotVelocityComponent(towerError - 90)) / 29);
        else setTurretAngle(0);
    }
    
    
    public static void turretPSAim(){
        turretPSAim(true);
    }
    
    
    public static void turretPSAim(boolean autoAim){
        if(Sensors.frontCamera.isHighGoalFound() && autoAim && Sensors.gyro.absAngleRange(30, 150) && getPower() > .1)
            setTurretAngle(Robot.getPSAngle() - Sensors.gyro.rawAngle() - 1 +
                                   (Sensors.robotVelocityComponent(Robot.getPSAngle() - Sensors.gyro.rawAngle() - 90)) / 41);
                                   //(Sensors.robotVelocityComponent(Robot.getPSAngle() - Sensors.gyro.rawAngle())) / 30);
        else setTurretAngle(0);
    }
    
    
    
    public static void feedRing(){ feeder.setPosition(RING_FEED); }
    
    public static void resetFeeder(){ feeder.setPosition(RESET); }
    
    public static void lockFeeder(){ feederLock.setPosition(FEEDER_LOCK); }
    
    public static void unlockFeeder(){ feederLock.setPosition(FEEDER_UNLOCK); }
    
    
    public static void setFeederCount(int feederCount){ feedCount = feederCount; }
    
    public static double feederCount(){ return feedCount; }
    
    
    public static void feederAutoState(int feedCount){
        feederState(Shooter.feedCount <= feedCount);
    }
    
    public static void feederState(boolean trigger){
        feederJustOn = false;
        switch (currentFeederState) {
            
            case IDLE:
                if(getPower() > .1 && getRPM() > targetRPM * .9 && trigger) newState(FeederState.FEED);
                
                if(feederTime.seconds() > LOCK_TIME){ lockFeeder(); isFeederLocked = true; }
                else{ unlockFeeder(); isFeederLocked = false; }
                
                resetFeeder();
                
                integralSumHigh = highGoalPID.integralSum;
                integralSumMid = midGoalPID.integralSum;
                break;
            
            case FEED:
                if (isFeederLocked) {
                    if (feederTime.seconds() > FEED_TIME + UNLOCK_TIME) {
                        newState(FeederState.RESET);
                    }
                    if (feederTime.seconds() > UNLOCK_TIME) feedRing();
                }else{
                    if ( feederTime.seconds() > FEED_TIME) {
                        newState(FeederState.RESET);
                    }
                    feedRing();
                }
                unlockFeeder();
                highGoalPID.setIntegralSum(integralSumHigh);
                midGoalPID.setIntegralSum(integralSumMid);
                break;
            
            case RESET:
                if (currentShooterState != ShooterState.POWER_SHOT && feederTime.seconds() > RESET_TIME + .03) { newState(FeederState.IDLE); feedCount++; break; }
                if (currentShooterState == ShooterState.POWER_SHOT && feederTime.seconds() > RESET_TIME + .03) { newState(FeederState.PS_DELAY); feederJustOn = true; feedCount++; break; }
                resetFeeder();
                unlockFeeder();
                highGoalPID.setIntegralSum(integralSumHigh);
                midGoalPID.setIntegralSum(integralSumMid);
                break;
            
            case PS_DELAY:
                if (feederTime.seconds() > PS_DELAY) { newState(FeederState.IDLE); break; }
                resetFeeder();
                unlockFeeder();
                highGoalPID.setIntegralSum(integralSumHigh);
                midGoalPID.setIntegralSum(integralSumMid);
                break;
        }
    }
    
    
    
    
    @RequiresApi(api = Build.VERSION_CODES.N)
    public static void highGoal(boolean autoPower, int rpmOffset){
        double towerDistance = Sensors.frontCamera.highGoalDistance();
        int RPM;
        rpmOffset += (curTarget == VisionUtils.Target.BLUE_GOAL) ? 40 : 0;
        if(towerDistance < 160 || !Sensors.frontCamera.isHighGoalFound() || !autoPower || !Sensors.gyro.absAngleRange(67.5, 127.5)) {
            RPM = TOP_GOAL;
        }else{
            RPM = (int) ((int) Math.pow(highMValue * (towerDistance - highKValue), 1.6) + highBValue) + rpmOffset;
        }
        setRPM(RPM, highGoalPID);
    }

    public static void highGoal(boolean autoPower){
        highGoal(autoPower, 0);
    }
    
    @RequiresApi(api = Build.VERSION_CODES.N)
    public static void midGoal(boolean autoPower){
        double towerDistance = Sensors.frontCamera.midGoalDistance();
        int RPM;
        int rpmOffset = (curTarget == VisionUtils.Target.BLUE_GOAL) ? 0 : -40;
        if(towerDistance < 160 || !Sensors.frontCamera.isMidGoalFound() || !autoPower || !Sensors.gyro.absAngleRange(67.5, 127.5)){
            RPM = MID_GOAL;
        }else{
            RPM = (int) ((int) Math.pow(midMValue * (towerDistance - midKValue), 1.6) + midBValue) + rpmOffset;
        }
        setRPM(RPM, midGoalPID);
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public static void powerShot(){
        Shooter.targetRPM = POWER_SHOT;
        setRPM(POWER_SHOT, powerShotPID);
    }
    
    
    @RequiresApi(api = Build.VERSION_CODES.N)
    public static void shooterState(boolean highGoal, boolean powerShot, boolean midGoal, boolean visionAim){
        shooterJustOn = false;
        switch (currentShooterState) {
            
            case OFF:
                if (highGoal) newState(ShooterState.HIGH_GOAL);
                if (midGoal) newState(ShooterState.MID_GOAL);
                if (powerShot) newState(ShooterState.POWER_SHOT);
                feedCount = 0;
                shooterOff();
                setTurretAngle(0);
                break;
                
            case HIGH_GOAL:
                if (powerShot) newState(ShooterState.POWER_SHOT);
                if (midGoal) newState(ShooterState.MID_GOAL);
                if (highGoal) newState(ShooterState.OFF);
                highGoal(visionAim);
                if(visionAim) turretAim();
                else setTurretAngle(0);
                break;
                
            case MID_GOAL:
                if (powerShot) newState(ShooterState.POWER_SHOT);
                if (midGoal) newState(ShooterState.OFF);
                if (highGoal) newState(ShooterState.HIGH_GOAL);
                midGoal(visionAim);
                setTurretAngle(0);
                break;
                
            case POWER_SHOT:
                if (highGoal) newState(ShooterState.HIGH_GOAL);
                if (midGoal) newState(ShooterState.MID_GOAL);
                if (powerShot) newState(ShooterState.OFF);
                powerShot();
                turretPSAim();
                break;
        }
    }
    
    
    //PID STUFF
    
    @RequiresApi(api = Build.VERSION_CODES.N)
    public static void setRPM(int targetRPM, PID pid) {
        Shooter.targetRPM = targetRPM;

        pid.setFComponent(targetRPM * PID_Constants.f);
        double shooterPower = pid.update(pow(targetRPM - updateRPM(), .7));

        multTelemetry.addData("shoter rom", shooterRPM);
        multTelemetry.addData("tornget", targetRPM);

        if (getRPM() < targetRPM * .92) {
            pid.setIntegralSum(targetRPM * .3);
        }
        shooterPower = Range.clip(shooterPower, 0.0, 1);
        setPower(shooterPower);

    }
    
    
    public static double updateRPM(){
        long currentTime = System.currentTimeMillis();
        long deltaMili = currentTime - timeRing.getValue(currentTime);
        double deltaMinutes = deltaMili / 60000.0;
        
        long currentPosition = getPosition();
        long deltaTicks = currentPosition - positionRing.getValue(currentPosition);
        double deltaRotations = deltaTicks / TICKS_PER_ROTATION;
        
        shooterRPM = Math.abs(deltaRotations / deltaMinutes);
        return shooterRPM;
    }
    
    public static double getRPM(){
        return shooterRPM;
    }
    
    
    
    //MOTOR CONTROL
    
    public static void setPower(double power){
        shooterFront.setPower(power);
        shooterBack.setPower(power);
    }
    
    @RequiresApi(api = Build.VERSION_CODES.N)
    public static void shooterOff(){ setPower(0.0); setRPM(0, highGoalPID); }
    
    public static double getPower(){
        return (shooterFront.getPower() + shooterBack.getPower()) / 2;
    }
    
    public static long getPosition(){
        return (shooterBack.getCurrentPosition() + shooterFront.getCurrentPosition()) / 2;
    }
    
    
    //STATE MACHINES
    
    private static void newState(FeederState newState) {
        currentFeederState = newState;
        feederTime.reset();
    }
    
    public static void newState(ShooterState newState) {
        if(currentShooterState == ShooterState.OFF){
            highGoalPID.resetIntegralSum();
            midGoalPID.resetIntegralSum();
            powerShotPID.resetIntegralSum();
            Intake.newState(Intake.IntakeState.OFF);
        }
        shooterJustOn = true;
        currentShooterState = newState; shooterTime.reset();
    }
    
    private enum FeederState {
        IDLE, RESET, FEED, PS_DELAY
    }
    
    public enum ShooterState {
        OFF, HIGH_GOAL, MID_GOAL, POWER_SHOT
    }
    

}
