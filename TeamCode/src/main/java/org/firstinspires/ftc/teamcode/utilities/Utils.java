package org.firstinspires.ftc.teamcode.utilities;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.Loggers.Clock;
import org.firstinspires.ftc.teamcode.utilities.Loggers.FileLogWriter;
import org.firstinspires.ftc.teamcode.utilities.Loggers.GridLogger;
import org.firstinspires.ftc.teamcode.utilities.Loggers.LogWriter;
import org.firstinspires.ftc.teamcode.utilities.Loggers.TestClock;

import static java.lang.Math.floorMod;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

public class Utils {

    public static HardwareMap hardwareMap;
    public static OpMode opMode;

    public static Telemetry telemetry;
    public static FtcDashboard dashboard = FtcDashboard.getInstance();
    public static Telemetry dashboardTelemetry = dashboard.getTelemetry();
    public static MultipleTelemetry multTelemetry;
    public static TelemetryPacket packet;

    // Writers
    public static LogWriter writer = new FileLogWriter("log.csv");
    public static Clock testClock = new TestClock();
    public static GridLogger towerGridLogger = new GridLogger(writer, testClock);

    // Writers
    public static LogWriter writer1 = new FileLogWriter("ring.csv");
    public static Clock testClock1 = new TestClock();
    public static GridLogger ringGridLogger = new GridLogger(writer1, testClock1);


    private static boolean isLinearOpMode;

    // Only use if it is in fact a LinearOpMode
    public static LinearOpMode linearOpMode = null;

    /**
     * Sets the OpMode
     * @param opMode
     */
    public static void setOpMode(OpMode opMode) {
        Utils.opMode = opMode;
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
        multTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
        packet = new TelemetryPacket();
        isLinearOpMode = (opMode instanceof LinearOpMode);
        if (isLinearOpMode) {
            linearOpMode = (LinearOpMode) opMode;
        }
    }


    public static boolean isActive(){
        if (isLinearOpMode) return linearOpMode.opModeIsActive();
        return true;
    }

    public static void breakpoint(boolean continue_on){
        while (!continue_on){
            multTelemetry.addLine("Break");
            multTelemetry.update();
        }
    }

    /**
     * I'm lazy
     * @param o
     */
    public static void print(Object o){
        System.out.println(o);
    }

}