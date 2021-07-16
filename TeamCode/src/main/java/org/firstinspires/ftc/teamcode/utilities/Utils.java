package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Utils {

    private static HardwareMap hardwareMap;

    public static HardwareMap hardwareMap(){
        return hardwareMap;
    }

    public static void setHardwareMap(HardwareMap hardwareMap){
        Utils.hardwareMap = hardwareMap;
    }
    
    public static void breakpoint(boolean continue_on){
        while(true){
            if(continue_on) break;
        }
    }

}
