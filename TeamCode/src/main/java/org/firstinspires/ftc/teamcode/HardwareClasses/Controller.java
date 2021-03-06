package org.firstinspires.ftc.teamcode.HardwareClasses;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utilities.Utils;

public class Controller {
	
	private final Gamepad gamepad;
	
	public Thumbstick rightStick, leftStick;
	public Button cross, circle, triangle, square, up, down, left, right, RB, LB, RT, LT, RS, LS, share, touchpad;
	public static Button touchSensor;

	public static DigitalChannel touchSensorObj;
	
	
	public Controller(Gamepad gamepad) {
		this.gamepad = gamepad;
		rightStick = new Thumbstick(); leftStick = new Thumbstick();
		
		cross = new Button(); circle = new Button(); triangle = new Button(); square = new Button();
		up = new Button(); down = new Button(); left = new Button(); right = new Button();
		RB = new Button(); LB = new Button(); RS = new Button(); LS = new Button();
		RT = new Button(); LT = new Button();
		share = new Button(); touchpad = new Button();

		touchSensorObj = Utils.hardwareMap.get(DigitalChannel.class, "touchsensor");
		touchSensorObj.setMode(DigitalChannel.Mode.INPUT);
		touchSensor = new Button();
	}
	
	
	public void update(){
		rightStick.update(gamepad.right_stick_x, gamepad.right_stick_y); leftStick.update(gamepad.left_stick_x, gamepad.left_stick_y);
		
		cross.update(gamepad.cross); circle.update(gamepad.circle); triangle.update(gamepad.triangle); square.update(gamepad.square);
		up.update(gamepad.dpad_up); down.update(gamepad.dpad_down); left.update(gamepad.dpad_left); right.update(gamepad.dpad_right);
		RB.update(gamepad.right_bumper); LB.update(gamepad.left_bumper); RS.update(gamepad.right_stick_button); LS.update(gamepad.left_stick_button);
		share.update(gamepad.share); touchpad.update(gamepad.touchpad);
		
		RT.update(gamepad.right_trigger); LT.update(gamepad.left_trigger);

		touchSensor.update(!touchSensorObj.getState());
	}
	
	
	public class Button {
		private boolean hold = false; private boolean press = false; private boolean toggle = false; private float rawVal = 0;
		private boolean inputYet = false;
		private int count = 0;

		private void update(float trigger){
			rawVal = trigger;
			update(rawVal > .5);
		}

		private void update(boolean button) {
			boolean wasHeld = hold;
			press = (hold = button) && !wasHeld;
			if (press) { count++; inputYet = true; }
		}

		public boolean inputYet(){ return inputYet; }
		
		public float rawVal() { return rawVal; }

		public boolean hold() { return hold; }
		
		public boolean press() { return press; }
		
		public boolean toggle() {
			if (press()) toggle = !toggle;
			return (toggle);
		}

		public void resetCount(){
			count = 0;
		}
		public int getCount(){
			return count;
		}

		public double range(double pressed, double released){
			double range = pressed - released;
			return (rawVal() * range) + released;
		}

	}

	
	
	public class Thumbstick {
		private double rawX, rawY, shiftedX, shiftedY;
		
		private void update(Float x, Float y) { rawX = x; rawY = y; }
		
		public double X() { return rawX * -1; }
		
		public double Y() { return rawY * -1; }
		
		
		public void setShift(double shiftAngle) {
			this.shiftedX = (this.rawX * Math.cos(Math.toRadians(shiftAngle))) - (this.rawY * Math.sin(Math.toRadians(shiftAngle)));
			this.shiftedY = (this.rawX * Math.sin(Math.toRadians(shiftAngle))) + (this.rawY * Math.cos(Math.toRadians(shiftAngle)));
		}
		
		public double shiftedX() { return shiftedX * -1; }
		
		public double shiftedY() { return shiftedY * -1; }
		
		
		public double getAngle(){ return ((270 - (Math.atan2(0 - Y(), 0 - X())) * 180 / Math.PI) % 360); }
	}
}
