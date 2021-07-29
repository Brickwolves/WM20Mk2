package org.firstinspires.ftc.teamcode.utilities.Loggers;

public interface LogWriter {
    void writeLine(String line);
    void stop();
    boolean isWriting();
}
