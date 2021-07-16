package org.firstinspires.ftc.teamcode.utilities;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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

    public static void log(String label, Object obj){
        TelemetryPacket packet = new TelemetryPacket();
        packet.put(label, obj);
    }

    public static void updateLog(){
        dashboard.sendTelemetryPacket(packet);
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