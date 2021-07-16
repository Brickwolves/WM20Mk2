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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareClasses.Controller;
import org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Gyro;
import org.firstinspires.ftc.teamcode.utilities.Utils;

import static java.lang.Math.abs;
import static java.lang.Math.max;


@TeleOp(name = "Chrids TeleOp", group = "TeleOp")
@Disabled
public class ChridsTeleOp extends OpMode {

  DcMotor frontLeft, frontRight, backLeft, backRight;
  Controller driver;
  static Gyro gyro = new Gyro();

  @Override
  public void init() {
    Utils.setHardwareMap(hardwareMap);
    
    frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
    frontRight = hardwareMap.get(DcMotor.class, "frontright");
    backLeft = hardwareMap.get(DcMotor.class, "backleft");
    backRight = hardwareMap.get(DcMotor.class, "backright");
  
    driver = new Controller(gamepad1);
  
    frontRight.setDirection(DcMotor.Direction.REVERSE);
    frontLeft.setDirection(DcMotor.Direction.FORWARD);
    backRight.setDirection(DcMotor.Direction.REVERSE);
    backLeft.setDirection(DcMotor.Direction.FORWARD);
  
    frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    
    gyro.init();
    
  }
  
  
  @Override
  public void init_loop() {
  }

  
  
  @Override
  public void start() {
    gyro.reset();
  }

  
  
  @Override
  public void loop() {
    driver.update();
    gyro.update();
    driver.rightStick.setShift(gyro.modAngle());
    
    double drive = driver.rightStick.Y();
    double turn = driver.leftStick.X();
    
    if(gamepad1.right_trigger > .4) {
       drive /= 2.5;
       turn /= 2.5;
    }
  
    drive = Range.clip(drive, -1, 1);
    turn = Range.clip(turn, -1, 1);
  
    double leftPower = drive - turn;
    double rightPower = drive + turn;
    
  
    double maxPower = abs(max(abs(leftPower), abs(rightPower)));
    if(maxPower > 1) { leftPower /= maxPower; rightPower /= maxPower; }
    else if(maxPower < .05 && maxPower > -.05) { leftPower = 0; rightPower = 0; }
  
    frontLeft.setPower(leftPower);
    frontRight.setPower(rightPower);
    backLeft.setPower(leftPower);
    backRight.setPower(rightPower);
    
  }
}
