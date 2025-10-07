// /*
//  * Copyright (c) 2021 OpenFTC Team
//  *
//  * Permission is hereby granted, free of charge, to any person obtaining a copy
//  * of this software and associated documentation files (the "Software"), to deal
//  * in the Software without restriction, including without limitation the rights
//  * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//  * copies of the Software, and to permit persons to whom the Software is
//  * furnished to do so, subject to the following conditions:
//  *
//  * The above copyright notice and this permission notice shall be included in all
//  * copies or substantial portions of the Software.
//  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//  * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//  * SOFTWARE.
//  */
// 
// package org.firstinspires.ftc.teamcode.old;
// 
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// 
// import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
// import org.openftc.apriltag.AprilTagDetection;
// import org.openftc.easyopencv.OpenCvCamera;
// import org.openftc.easyopencv.OpenCvCameraFactory;
// import org.openftc.easyopencv.OpenCvCameraRotation;
// import org.openftc.easyopencv.OpenCvInternalCamera;
// 
// import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.DcMotorEx;
// import com.qualcomm.robotcore.hardware.DcMotorSimple;
// import com.qualcomm.robotcore.hardware.IMU;
// import com.qualcomm.robotcore.hardware.LED;
// import com.qualcomm.robotcore.hardware.Servo;
// import com.qualcomm.robotcore.util.ElapsedTime;
// import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
// import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
// import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
// 
// import java.util.ArrayList;
// 
// @Autonomous
// public class Autonomous_Left_State_Copy extends LinearOpMode
// {
//	 OpenCvCamera camera;
//	 AprilTagDetectionPipeline aprilTagDetectionPipeline;
// 
//	 static final double FEET_PER_METER = 3.28084;
// 
//	 private IMU imu_IMU;
//	 private LED LED_LED;
//	 private DcMotor FrontRightWheel;
//	 private DcMotor BackRightWheel;
//	 private DcMotor FrontLeftWheel;
//	 private DcMotor BackLeftWheel;
//	 private Servo LeftDownServo;
//	 private Servo RotateServo2;
//	 private DcMotor FrontLeftArm;
//	 private DcMotor ArmExtendor;
//	 private DcMotor FrontRightArm;
//	 private DcMotor Stopper;
//	 private Servo GrabServo;
//	 private Servo RotateServo1;
//	 private Servo CameraServo;
//	 private Servo RightDownServo;
// 
//	 int Cones_Dropped;
//	 ElapsedTime Current_Time;
//	 int Step;
//	 double Time_Till_Arm_Raise;
// 
//	 // Lens intrinsics
//	 // UNITS ARE PIXELS
//	 // NOTE: this calibration is for the C920 webcam at 800x448.
//	 // You will need to do your own calibration for other configurations!
//	 double fx = 578.272;
//	 double fy = 578.272;
//	 double cx = 402.145;
//	 double cy = 221.506;
// 
//	 // UNITS ARE METERS
//	 double tagsize = 0.166;
// 
//	  // Tag ID 1,2,3 from the 36h11 family
//	 int LEFT = 1;
//	 int MIDDLE = 2;
//	 int RIGHT = 3;
//	 
//	 AprilTagDetection tagOfInterest = null;
// 
//	 @Override
//	 public void runOpMode()
//	 {
//	   
//		 int Time_Till_Park;
//		 int Desired_Location;
//	 
//		 imu_IMU = hardwareMap.get(IMU.class, "imu");
//		 LED_LED = hardwareMap.get(LED.class, "LED");
//		 FrontRightWheel = hardwareMap.get(DcMotor.class, "Front-Right-Wheel");
//		 BackRightWheel = hardwareMap.get(DcMotor.class, "Back-Right-Wheel");
//		 FrontLeftWheel = hardwareMap.get(DcMotor.class, "Front-Left-Wheel");
//		 BackLeftWheel = hardwareMap.get(DcMotor.class, "Back-Left-Wheel");
//		 LeftDownServo = hardwareMap.get(Servo.class, "Left-Down-Servo");
//		 RotateServo2 = hardwareMap.get(Servo.class, "RotateServo2");
//		 FrontLeftArm = hardwareMap.get(DcMotor.class, "Front-Left-Arm");
//		 ArmExtendor = hardwareMap.get(DcMotor.class, "Arm-Extendor");
//		 FrontRightArm = hardwareMap.get(DcMotor.class, "Front-Right-Arm");
//		 Stopper = hardwareMap.get(DcMotor.class, "Stopper");
//		 GrabServo = hardwareMap.get(Servo.class, "Grab-Servo");
//		 RotateServo1 = hardwareMap.get(Servo.class, "RotateServo1");
//		 CameraServo = hardwareMap.get(Servo.class, "Camera-Servo");
//		 RightDownServo = hardwareMap.get(Servo.class, "Right-Down-Servo");
//	 
//		 // Initialize the IMU.
//		 // Initializes the IMU with non-default settings. To use this block,
//		 // plug one of the "new IMU.Parameters" blocks into the parameters socket.
//		 // Creates a Parameters object for use with an IMU in a REV Robotics Control Hub or Expansion Hub, specifying the hub's orientation on the robot via the direction that the REV Robotics logo is facing and the direction that the USB ports are facing.
//		 imu_IMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
//		 // Initialize Motors and Servos
//		 Initialization();
//		 // Set Variables
//		 Step = 0;
//		 Time_Till_Park = 25;
//		 Time_Till_Arm_Raise = 0.5;
//		 Cones_Dropped = 0;
//		 Desired_Location = 0;
//		 // Disable LED
//		 LED_LED.enable(false);
//		 // Prompt user to press start button.
//		 telemetry.addData("IMU Example", "Press start to continue...");
//		 imu_IMU.resetYaw();
//		 telemetry.update();
//	   
//		 int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//		 camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//		 aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
// 
//		 camera.setPipeline(aprilTagDetectionPipeline);
//		 camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//		 {
//			 @Override
//			 public void onOpened()
//			 {
//				 camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
//			 }
// 
//			 @Override
//			 public void onError(int errorCode)
//			 {
// 
//			 }
//		 });
// 
//		 telemetry.setMsTransmissionInterval(50);
//		 
//		 /*
//		  * The INIT-loop:
//		  * This REPLACES waitForStart!
//		  */
//		 while (!isStarted() && !isStopRequested())
//		 {
//			 ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
// 
//			 if(currentDetections.size() != 0)
//			 {
//				 boolean tagFound = false;
// 
//				 for(AprilTagDetection tag : currentDetections)
//				 {
//					 if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
//					 {
//						 tagOfInterest = tag;
//						 Desired_Location = tag.id;
//						 tagFound = true;
//						 break;
//					 }
//				 }
// 
//				 if(tagFound)
//				 {
//					 telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
//					 tagToTelemetry(tagOfInterest);
//				 }
//				 else
//				 {
//					 telemetry.addLine("Don't see tag of interest :(");
// 
//					 if(tagOfInterest == null)
//					 {
//						 telemetry.addLine("(The tag has never been seen)");
//					 }
//					 else
//					 {
//						 telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//						 tagToTelemetry(tagOfInterest);
//					 }
//				 }
// 
//			 }
//			 else
//			 {
//				 telemetry.addLine("Don't see tag of interest :(");
// 
//				 if(tagOfInterest == null)
//				 {
//					 telemetry.addLine("(The tag has never been seen)");
//				 }
//				 else
//				 {
//					 telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//					 tagToTelemetry(tagOfInterest);
//				 }
// 
//			 }
// 
//			 telemetry.update();
//			 sleep(20);
//		 }
// 
//		 /*
//		  * The START command just came in: now work off the latest snapshot acquired
//		  * during the init loop.
//		  */
// 
//		 /* Update the telemetry */
//		 if(tagOfInterest != null)
//		 {
//			 telemetry.addLine("Tag snapshot:\n");
//			 tagToTelemetry(tagOfInterest);
//			 telemetry.update();
//		 }
//		 else
//		 {
//			 telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
//			 telemetry.update();
//		 }
// 
//	 // Start Internal Timer for Parking
//	   (new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS)).reset();
//	   Current_Time = new ElapsedTime(System.nanoTime());
//	   Drive_System(0, 0, 2525, 0.5, 1); //2500
//	   Drive_System(3, -73.45, 0, 0, 1);
//	   RotateServo1.setPosition(0.8);
//	   RotateServo2.setPosition(0.8);
//	   Arm_Up();
//	   Drop_Cone();
//	   Collect_Cone();
//	   Arm_Up();
//	   Drop_Cone();
//	   Collect_Cone();
//	   Arm_Up();
//	   Drop_Cone();
//	   Collect_Cone();
//	   Arm_Up();
//	   Drop_Cone();
//	   Collect_Cone();
//	   Arm_Up();
//	   Drop_Cone();
//	   Collect_Cone();
//	   Arm_Up();
//	   Drop_Cone();
//	   Reset();
//	   GrabServo.setPosition(0);
//	   RotateServo1.setPosition(0.9);
//	   RotateServo2.setPosition(0.9);
//	   FrontRightWheel.setPower(-0.5);
//	   BackRightWheel.setPower(0.5);
//	   FrontLeftWheel.setPower(0.5);
//	   BackLeftWheel.setPower(-0.5);
//	   sleep(350);
//	   BackLeftWheel.setPower(0);
//	   BackRightWheel.setPower(0);
//	   FrontLeftWheel.setPower(0);
//	   FrontRightWheel.setPower(0);
//	   Drive_System(1, -90, 0, 0, 1);
//	   telemetry.addData("Desired Location", Desired_Location);
//	   telemetry.update();
//	   sleep(750);
// 
//		 /* Actually do something useful */
//		 if(Desired_Location == 1){
//			 FrontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//			 BackRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//			 FrontLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//			 BackLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//			 FrontRightWheel.setPower(-0.5);
//			 BackRightWheel.setPower(-0.5);
//			 FrontLeftWheel.setPower(-0.5);
//			 BackLeftWheel.setPower(-0.5);
//			 FrontRightWheel.setTargetPosition(-700);
//			 BackRightWheel.setTargetPosition(-700);
//			 FrontLeftWheel.setTargetPosition(-700);
//			 BackLeftWheel.setTargetPosition(-700);
//			 FrontRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//			 BackRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//			 FrontLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//			 BackLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//		 }else if(Desired_Location == 2){
//			 FrontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//			 BackRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//			 FrontLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//			 BackLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//			 FrontRightWheel.setPower(0.5);
//			 BackRightWheel.setPower(0.5);
//			 FrontLeftWheel.setPower(0.5);
//			 BackLeftWheel.setPower(0.5);
//			 FrontRightWheel.setTargetPosition(425);
//			 BackRightWheel.setTargetPosition(425);
//			 FrontLeftWheel.setTargetPosition(425);
//			 BackLeftWheel.setTargetPosition(425);
//			 FrontRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//			 BackRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//			 FrontLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//			 BackLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//		 }else if(Desired_Location == 3){
//			 FrontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//			 BackRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//			 FrontLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//			 BackLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//			 FrontRightWheel.setPower(0.5);
//			 BackRightWheel.setPower(0.5);
//			 FrontLeftWheel.setPower(0.5);
//			 BackLeftWheel.setPower(0.5);
//			 FrontRightWheel.setTargetPosition(1550);
//			 BackRightWheel.setTargetPosition(1550);
//			 FrontLeftWheel.setTargetPosition(1550);
//			 BackLeftWheel.setTargetPosition(1550);
//			 FrontRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//			 BackRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//			 FrontLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//			 BackLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//		 }
//		 sleep(5000);
// 
// 
//		 /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
//	 }
// 
//	 void tagToTelemetry(AprilTagDetection detection)
//	 {
//		 telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
//		 telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
//		 telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
//		 telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
//		 telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
//		 telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
//		 telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
//	 }
//	 
//	 /**
//	* Describe this function...
//	*/
//   private void Initialization() {
//	 LeftDownServo.setDirection(Servo.Direction.REVERSE);
//	 FrontRightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
//	 BackRightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
//	 FrontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//	 FrontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//	 BackRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//	 BackLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//	 RotateServo2.setDirection(Servo.Direction.REVERSE);
//	 FrontLeftArm.setDirection(DcMotorSimple.Direction.REVERSE);
//	 ArmExtendor.setDirection(DcMotorSimple.Direction.FORWARD);
//	 ArmExtendor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//	 ArmExtendor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	 FrontLeftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//	 FrontRightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//	 FrontLeftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	 FrontRightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	 Stopper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//	 Stopper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	 ((DcMotorEx) ArmExtendor).setTargetPositionTolerance(25);
//	 ((DcMotorEx) FrontRightArm).setTargetPositionTolerance(35);
//	 ((DcMotorEx) FrontLeftArm).setTargetPositionTolerance(35);
//	 GrabServo.setPosition(0);
//	 RotateServo1.setPosition(0.9);
//	 RotateServo2.setPosition(0.9);
//	 CameraServo.setPosition(0.61);
//	 LeftDownServo.setDirection(Servo.Direction.REVERSE);
//	 LeftDownServo.setPosition(0.375);
//	 sleep(1000);
//	 RightDownServo.setPosition(0.26);
//   }
// 
//   /**
//	* Describe this function...
//	*/
//   private void Drive_System(int Mode, double Desired_Angle, int Desired_Distance, double Power, int Polarity) {
//	 Orientation Current_Angle;
//	 double Angle_Error;
// 
//	 if (Polarity == 1) {
//	   // Wheels Forward
//	   FrontRightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
//	   BackRightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
//	   FrontLeftWheel.setDirection(DcMotorSimple.Direction.FORWARD);
//	   BackLeftWheel.setDirection(DcMotorSimple.Direction.FORWARD);
//	 } else if (Polarity == -1) {
//	   // Wheels Backwards
//	   FrontRightWheel.setDirection(DcMotorSimple.Direction.FORWARD);
//	   BackRightWheel.setDirection(DcMotorSimple.Direction.FORWARD);
//	   FrontLeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
//	   BackLeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
//	 }
//	 Current_Angle = imu_IMU.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
//	 Angle_Error = Desired_Angle - Current_Angle.thirdAngle;
//	 if (Mode == 0) {
//	   FrontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	   FrontLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	   FrontRightWheel.setTargetPosition(Desired_Distance);
//	   FrontLeftWheel.setTargetPosition(Desired_Distance);
//	   FrontRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	   FrontLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	   while (!(Desired_Distance - Math.abs(FrontRightWheel.getCurrentPosition()) < 25)) {
//		 if (Current_Time.time() >= Time_Till_Arm_Raise && Step == 0) {
//		   Drive_Position();
//		   Step += 1;
//		 }
//		 Current_Angle = imu_IMU.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
//		 Angle_Error = Desired_Angle - Current_Angle.thirdAngle;
//		 FrontRightWheel.setPower(Power);
//		 FrontLeftWheel.setPower(Power + Angle_Error / 5);
//		 BackRightWheel.setPower(FrontRightWheel.getPower());
//		 BackLeftWheel.setPower(FrontLeftWheel.getPower());
//		 telemetry.addData("Mode", "Drive");
//		 telemetry.addData("Current Angle", Current_Angle.thirdAngle);
//		 telemetry.addData("Angle Displacement", Desired_Angle - Current_Angle.thirdAngle);
//		 telemetry.addData("Target Displacement", FrontRightWheel.getTargetPosition() - Math.abs(FrontRightWheel.getCurrentPosition()));
//		 telemetry.update();
//	   }
//	 } else if (Mode == 1) {
//	   FrontRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//	   FrontLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//	   BackRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//	   BackLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//	   while (!(Math.abs(Angle_Error) < 0.5)) {
//		 Current_Angle = imu_IMU.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
//		 Angle_Error = Desired_Angle - Current_Angle.thirdAngle;
//		 FrontRightWheel.setPower(Math.min(Math.max(-(Power + Angle_Error / 20), -0.5), 0.5));
//		 FrontLeftWheel.setPower(Math.min(Math.max(Power + Angle_Error / 20, -0.5), 0.5));
//		 BackRightWheel.setPower(Math.min(Math.max(-(Power + Angle_Error / 20), -0.5), 0.5));
//		 BackLeftWheel.setPower(Math.min(Math.max(Power + Angle_Error / 20, -0.5), 0.5));
//		 telemetry.addData("Mode", "Turning");
//		 telemetry.addData("Current Angle", Current_Angle.thirdAngle);
//		 telemetry.addData("Angle Displacement", Desired_Angle - Current_Angle.thirdAngle);
//		 telemetry.addData("Target Displacement", FrontRightWheel.getTargetPosition() - Math.abs(FrontRightWheel.getCurrentPosition()));
//		 telemetry.update();
//	   }
//	 } else if (Mode == 2) {
//	 } else if (Mode == 3) {
//	   FrontRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//	   FrontLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//	   BackRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//	   BackLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//	   while (!(Math.abs(Angle_Error) < 0.2)) {
//		 Current_Angle = imu_IMU.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
//		 Angle_Error = Desired_Angle - Current_Angle.thirdAngle;
//		 FrontRightWheel.setPower(Math.min(Math.max(-(Power + Angle_Error / 8), -0.5), 0.5));
//		 FrontLeftWheel.setPower(Math.min(Math.max(Power + Angle_Error / 8, -0.5), 0.5));
//		 BackRightWheel.setPower(0);
//		 BackLeftWheel.setPower(Math.min(Math.max(Power + Angle_Error / 8, -0.5), 0.5));
//		 telemetry.addData("Mode", "Turning");
//		 telemetry.addData("Current Angle", Current_Angle.thirdAngle);
//		 telemetry.addData("Angle Displacement", Desired_Angle - Current_Angle.thirdAngle);
//		 telemetry.addData("Target Displacement", FrontRightWheel.getTargetPosition() - Math.abs(FrontRightWheel.getCurrentPosition()));
//		 telemetry.update();
//	   }
//	 }
//	 FrontRightWheel.setPower(0);
//	 FrontLeftWheel.setPower(0);
//	 BackRightWheel.setPower(0);
//	 BackLeftWheel.setPower(0);
//   }
// 
//   /**
//	* Describe this function...
//	*/
//   private void Reset() {
//	 FrontRightArm.setTargetPosition(100);
//	 FrontLeftArm.setTargetPosition(100);
//	 FrontRightArm.setPower(0.5);
//	 FrontLeftArm.setPower(0.5);
//	 FrontRightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	 FrontLeftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	 ArmExtendor.setTargetPosition(0);
//	 ArmExtendor.setPower(1);
//	 ArmExtendor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//   }
// 
//   /**
//	* Describe this function...
//	*/
//   private void Drive_Position() {
//	 FrontRightArm.setTargetPosition(1900);
//	 FrontLeftArm.setTargetPosition(1900);
//	 FrontRightArm.setPower(0.5);
//	 FrontLeftArm.setPower(0.5);
//	 FrontRightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	 FrontLeftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	 ArmExtendor.setTargetPosition(2950);
//	 ArmExtendor.setPower(0.5);
//	 ArmExtendor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//   }
// 
//   /**
//	* Describe this function...
//	*/
//   private void Arm_Up() {
//	 FrontRightArm.setTargetPosition(2500);
//	 FrontLeftArm.setTargetPosition(2500);
//	 FrontRightArm.setPower(0.4);
//	 FrontLeftArm.setPower(0.4);
//	 if (Cones_Dropped != 0) {
//	   sleep(650);
//	   RotateServo1.setPosition(0.8);
//	   RotateServo2.setPosition(0.8);
//	 }
//	 while (FrontRightArm.getCurrentPosition() < 2350) {
//	 }
//   }
// 
//   /**
//	* Describe this function...
//	*/
//   private void Drop_Cone() {
//	 GrabServo.setPosition(0.75);
//	 Cones_Dropped += 1;
//	 sleep(500);
//	 GrabServo.setPosition(0);
//   }
// 
//   /**
//	* Describe this function...
//	*/
//   private void Collect_Cone() {
//	 RotateServo1.setPosition(0.035);
//	 RotateServo2.setPosition(0.035);
//	 if (Cones_Dropped == 1) {
//	   FrontRightArm.setTargetPosition(500 - Cones_Dropped * 50);
//	   FrontLeftArm.setTargetPosition(500 - Cones_Dropped * 50);
//	   FrontRightArm.setPower(0.75);
//	   FrontLeftArm.setPower(0.75);
//	 } else if (Cones_Dropped == 2) {
//	   FrontRightArm.setTargetPosition(500 - Cones_Dropped * 50);
//	   FrontLeftArm.setTargetPosition(500 - Cones_Dropped * 50);
//	   FrontRightArm.setPower(0.75);
//	   FrontLeftArm.setPower(0.75);
//	 } else if (Cones_Dropped == 3) {
//	   FrontRightArm.setTargetPosition(500 - Cones_Dropped * 50);
//	   FrontLeftArm.setTargetPosition(500 - Cones_Dropped * 50);
//	   FrontRightArm.setPower(0.75);
//	   FrontLeftArm.setPower(0.75);
//	 } else if (Cones_Dropped == 4) {
//	   FrontRightArm.setTargetPosition(500 - Cones_Dropped * 50);
//	   FrontLeftArm.setTargetPosition(500 - Cones_Dropped * 50);
//	   FrontRightArm.setPower(0.75);
//	   FrontLeftArm.setPower(0.75);
//	 } else if (Cones_Dropped == 5) {
//	   FrontRightArm.setTargetPosition(500 - Cones_Dropped * 50);
//	   FrontLeftArm.setTargetPosition(500 - Cones_Dropped * 50);
//	   FrontRightArm.setPower(0.75);
//	   FrontLeftArm.setPower(0.75);
//	 }
//	 while (500 - Cones_Dropped * 50 < FrontRightArm.getCurrentPosition() - 20) {
//	   if ((500 - Cones_Dropped * 50) + 400 >= FrontRightArm.getCurrentPosition()) {
//		 telemetry.addData("Rotating", 1);
//		 telemetry.update();
//		 if (Cones_Dropped == 1) {
//		   RotateServo1.setPosition(0.05);
//		   RotateServo2.setPosition(0.05);
//		 } else if (Cones_Dropped == 2) {
//		   RotateServo1.setPosition(0.07);
//		   RotateServo2.setPosition(0.07);
//		 } else if (Cones_Dropped == 3) {
//		   RotateServo1.setPosition(0.09);
//		   RotateServo2.setPosition(0.09);
//		 } else if (Cones_Dropped == 4) {
//		   RotateServo1.setPosition(0.11);
//		   RotateServo2.setPosition(0.11);
//		 } else if (Cones_Dropped == 5) {
//		   RotateServo1.setPosition(0.13);
//		   RotateServo2.setPosition(0.13);
//		 }
//	   }
//	 }
//   }
// 
// }
// 
