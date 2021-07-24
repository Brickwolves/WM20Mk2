/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.HardwareClasses.Controller;
import org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.AimBotPipe;
import org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.CameraV2;
import org.firstinspires.ftc.teamcode.HardwareClasses.Sensors;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static com.qualcomm.robotcore.util.Range.clip;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.TOWER_AUTO_CALIBRATE_ON;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.curTarget;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_Sanic.ONE_RING_HEIGHT;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_Sanic.RING_AUTO_CALIBRATE_ON;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_Sanic.RING_MAX_THRESH;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_Sanic.RING_MIN_THRESH;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.Target.BLUE_GOAL;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.Target.RED_GOAL;
import static org.firstinspires.ftc.teamcode.HardwareClasses.Sensors.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.utilities.Utils.multTelemetry;
import static org.firstinspires.ftc.teamcode.utilities.Utils.setOpMode;

@TeleOp(name = "RingTest", group = "Concept")
public class RingTest extends OpMode {

  private ElapsedTime runtime = new ElapsedTime();

  private OpenCvCamera webcam;
  private CameraV2 cam;
  private Controller controller;
  private DcMotor fl, fr, bl, br;

  @Override
  public void init() {
    setOpMode(this);

    controller = new Controller(gamepad1);
    cam = new CameraV2("Back Camera", true);

    initMotors();

  }

  public void initMotors(){
    fl = hardwareMap.get(DcMotor.class, "frontleft");
    fr = hardwareMap.get(DcMotor.class, "frontright");
    bl = hardwareMap.get(DcMotor.class, "backleft");
    br = hardwareMap.get(DcMotor.class, "backright");

    fr.setDirection(DcMotorSimple.Direction.FORWARD);
    fl.setDirection(DcMotorSimple.Direction.REVERSE);
    br.setDirection(DcMotorSimple.Direction.FORWARD);
    bl.setDirection(DcMotorSimple.Direction.REVERSE);

    fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  }

  public void setPowerTele(double y, double x, double r, double p){
    fr.setPower((y - x - r) * p);
    fl.setPower((y + x + r) * p);
    br.setPower((y + x - r) * p);
    bl.setPower((y - x - r) * p);
  }


  /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
  @Override
  public void init_loop() {
    controls();
  }

  /*
   * This method will be called ONCE when start is pressed
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  @Override
  public void start() {
    runtime.reset();
  }

  /*
   * This method will be called repeatedly in a loop
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  @Override
  public void loop() {
    controls();
  }

  public void controls(){
    controller.update();

    cam.calibrateRingDetection(controller.square.press());

    multTelemetry.addData("Status", "Run Time: " + runtime.toString());
    multTelemetry.addData("Mode", (RING_AUTO_CALIBRATE_ON) ? "Auto Calibration": "Detecting");
    multTelemetry.addData("Ring Count", cam.startingStackCount());
    multTelemetry.addData("RING MAX", RING_MAX_THRESH);
    multTelemetry.addData("RING MIN", RING_MIN_THRESH);
    multTelemetry.addData("Stack Height", cam.sanicPipe.getRingRect().height);
    multTelemetry.addData("ONE RING HEIGHT", ONE_RING_HEIGHT);

    multTelemetry.update();
  }

}
