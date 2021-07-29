package org.firstinspires.ftc.teamcode.utilities.Loggers;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import static org.firstinspires.ftc.teamcode.utilities.Loggers.Dash_Reader.FILE_NAME;
import static org.firstinspires.ftc.teamcode.utilities.Loggers.Dash_Reader.LOG_DIR;

public class FileLogWriter implements LogWriter {

    private String fileName;
    private boolean writing = false;
    private FileWriter fileWriter;

    public FileLogWriter(String fileName) {
        this.fileName = fileName;
    }

    @Override
    public void writeLine(String line) {
        if (!writing) {
            openFile();
            writing = true;
        }

        try {
            fileWriter.append(line);
            fileWriter.append("\n");
            fileWriter.flush();
        } catch (IOException e) {
            throw new Error(e);
        }

    }

    private void openFile() {
        try {
            File file = new File(LOG_DIR + FILE_NAME);
            if (file.exists()) {
                file.delete();
            }
            file.createNewFile();
            fileWriter = new FileWriter(file);
            fileWriter.flush();

        } catch (IOException e) {
            throw new Error(e);
        }
    }

    private void closeFile() {
        writing = false;
        try{
            fileWriter.close();
        } catch (IOException e) {
            throw new Error(e);
        }
    }

    @Override
    public void stop() {
        if (writing) {
            closeFile();
        }
    }
}

