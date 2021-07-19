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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.HardwareClasses.Controller;
import org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.OpCuttleFish.CuttleFish;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static com.qualcomm.robotcore.util.Range.clip;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.OpCuttleFish.Dash_CuttleFish.cuttle_x;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.OpCuttleFish.Dash_CuttleFish.cuttle_y;
import static org.firstinspires.ftc.teamcode.utilities.Utils.multTelemetry;
import static org.firstinspires.ftc.teamcode.utilities.Utils.setOpMode;

@TeleOp(name = "VisionTest", group = "Concept")
public class VisionTest extends OpMode {

  private ElapsedTime runtime = new ElapsedTime();

  private OpenCvCamera webcam;
  private CuttleFish cuttleFish = new CuttleFish();
  private Controller controller;
  private DcMotor fl, fr, bl, br;

  @Override
  public void init() {
    setOpMode(this);


    controller = new Controller(gamepad1);

    initMotors();
    initCameras();

  }
  public void initCameras(){
    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Front Camera"), cameraMonitorViewId);
    webcam.openCameraDeviceAsync(() -> webcam.startStreaming(432, 240, OpenCvCameraRotation.UPRIGHT));
    webcam.setPipeline(cuttleFish);
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

    double power = clip(gamepad1.right_trigger, 0.3, 0.8);

    cuttle_x += controller.rightStick.X();
    cuttle_y += controller.rightStick.Y();



    setPowerTele(controller.rightStick.Y(), controller.rightStick.X(), controller.leftStick.X(), power);

    multTelemetry.addData("Status", "Run Time: " + runtime.toString());
    multTelemetry.update();
  }

}
