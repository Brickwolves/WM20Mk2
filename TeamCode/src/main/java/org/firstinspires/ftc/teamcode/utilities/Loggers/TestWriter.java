package org.firstinspires.ftc.teamcode.utilities.Loggers;

import java.util.ArrayList;
import java.util.List;

public class TestWriter implements LogWriter {

    private List<String> lines = new ArrayList<>();
    private boolean stopCalled;

    public TestWriter(){
        stopCalled = false;
    }

    public void writeLine(String line){
        lines.add(line);
    }

    public List<String> getLines(){
        return lines;
    }

    public void stop(){
        stopCalled = true;
    }

    @Override
    public boolean isWriting() {
        return stopCalled;
    }

    public boolean isStopCalled(){
        return stopCalled;
    }
}
