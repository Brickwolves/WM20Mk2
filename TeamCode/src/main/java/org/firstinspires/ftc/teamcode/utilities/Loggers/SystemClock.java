package org.firstinspires.ftc.teamcode.utilities.Loggers;

public class SystemClock implements Clock {
    private long startTime;
    public SystemClock(){
        startTime = System.currentTimeMillis();
    }

    public long getCurrentTime(){
        return System.currentTimeMillis();
    }
    public double getTimePassed(){
        return (getCurrentTime() - startTime);
    }

}
