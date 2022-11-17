package org.firstinspires.ftc.teamcode.extra;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.main.LastKey;
import org.firstinspires.ftc.teamcode.main.Lift;
import org.firstinspires.ftc.teamcode.main.Main;

@Disabled
@TeleOp(name="Linear-Slide-Test", group="Power-Play-Extras")
public class Linear_Slide_Test extends Main {
 LastKey lastkey;
 private Lift lift = null;

 @Override
 public void init() {

  // Initialize lift
  lift = new Lift(hardwareMap.get(DcMotorEx.class,"lift"));

  // Start recording last inputs
  lastkey = new LastKey(gamepad1);

 }

 @Override
 public void init_loop() {

 }

 @Override
 public void start() {

 }

 @Override
 public void loop() {
  if(gamepad1.dpad_up){
   lift.motor.setVelocity(10);
  }
  if(gamepad1.dpad_down){
   lift.motor.setVelocity(-10);
  }
 }
 @Override
 public void stop() {

 }

}
