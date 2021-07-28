package org.firstinspires.ftc.teamcode.utilities.Loggers;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;

import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.BLUE_MAX_THRESH;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.BLUE_MIN_THRESH;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.RED_MAX_THRESH;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.RED_MIN_THRESH;
import static org.firstinspires.ftc.teamcode.utilities.Loggers.Dash_Reader.LOG_DIR;

public class CSVReader
{

    public static void reloadThresholds(){
        String splitBy = ",";
        try
        {
            //parsing a CSV file into BufferedReader class constructor
            BufferedReader br = new BufferedReader(new FileReader(LOG_DIR));

            // header
            // hsv max
            // hsv min
            // ycrcb max
            // ycrcb min
            // each row has four items, last one is time


            String headers = br.readLine();

            // update blue goal max hsv
            String[] max_blue_thresh = br.readLine().split(splitBy);
            for (int i=0; i < max_blue_thresh.length - 1; i++){
                BLUE_MAX_THRESH.val[i] = Double.parseDouble(max_blue_thresh[i]);
            }

            // update blue goal min hsv
            String[] min_blue_thresh = br.readLine().split(splitBy);
            for (int i=0; i < min_blue_thresh.length - 1; i++){
                BLUE_MIN_THRESH.val[i] = Double.parseDouble(min_blue_thresh[i]);
            }

            // update blue goal max hsv
            String[] max_red_thresh = br.readLine().split(splitBy);
            for (int i=0; i < max_red_thresh.length - 1; i++){
                RED_MAX_THRESH.val[i] = Double.parseDouble(max_red_thresh[i]);
            }

            // update blue goal min hsv
            String[] min_red_thresh = br.readLine().split(splitBy);
            for (int i=0; i < min_red_thresh.length - 1; i++){
                RED_MIN_THRESH.val[i] = Double.parseDouble(min_red_thresh[i]);
            }
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
    }

    public void readCSV()
    {
        String line = "";
        String splitBy = ",";
        try
        {
            //parsing a CSV file into BufferedReader class constructor
            BufferedReader br = new BufferedReader(new FileReader(LOG_DIR));
            int num_line = 0;
            while ((line = br.readLine()) != null)   //returns a Boolean value
            {
                String[] thresh_values = line.split(splitBy);    // use comma as separator

                // First line will be thresholds
                System.out.println("THRESH [ONE=" + thresh_values[0] + ", TWO=" + thresh_values[1] + ", THREE=" + thresh_values[2] + ", TIME=" + thresh_values[3] + "]");

                num_line++;
            }
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
    }
}