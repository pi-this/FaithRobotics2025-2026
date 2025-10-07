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
// import com.qualcomm.hardware.bosch.BNO055IMU;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.DcMotorEx;
// import com.qualcomm.robotcore.hardware.DcMotorSimple;
// import com.qualcomm.robotcore.hardware.LED;
// import com.qualcomm.robotcore.hardware.Servo;
// import java.util.List;
// import org.firstinspires.ftc.robotcore.external.JavaUtil;
// import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
// import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
// import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
// import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
// import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
// 
// import java.util.ArrayList;
// 
// @Autonomous
// public class AprilTagAutonomousInitDetectionExample_Copy extends LinearOpMode
// {
//	 private VuforiaCurrentGame vuforiaPOWERPLAY;
//   private DcMotor FrontRightWheel;
//   private DcMotor BackRightWheel;
//   private Servo RotateServo2;
//   private DcMotor FrontLeftArm;
//   private Servo GrabServo;
//   private Servo RotateServo1;
//   private Servo CameraServo;
//   private DcMotor ArmExtendor;
//   private DcMotor FrontRightArm;
//   private DcMotor Stopper;
//   private LED LED_LED;
//   private DcMotor FrontLeftWheel;
//   private DcMotor BackLeftWheel;
//   private Servo LeftDownServo;
//   private Servo RightDownServo;
//   private BNO055IMU imu;
// 
//   boolean isVisible;
//   VuforiaBase.TrackingResults vuforiaResults;
//   double Last_Position_X;
//   int Cones_Dropped;
//   int Step;
//   int Determined_Location;
//   boolean In_Position;
// 
//   /**
//	* Describe this function...
//	*/
//   private void Initialization() {
//	 BNO055IMU.Parameters IMU2;
// 
//	 IMU2 = new BNO055IMU.Parameters();
//	 telemetry.addData("Status", "Initializing Vuforia. Please wait...");
//	 telemetry.update();
//	 vuforiaPOWERPLAY.initialize(
//		 "", // vuforiaLicenseKey
//		 hardwareMap.get(WebcamName.class, "Webcam 2"), // cameraName
//		 "", // webcamCalibrationFilename
//		 false, // useExtendedTracking
//		 true, // enableCameraMonitoring
//		 VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES, // cameraMonitorFeedback
//		 0, // dx
//		 0, // dy
//		 0, // dz
//		 AxesOrder.XZY, // axesOrder
//		 90, // firstAngle
//		 90, // secondAngle
//		 0, // thirdAngle
//		 true); // useCompetitionFieldTargetLocations
//	 //vuforiaPOWERPLAY.activate();
//	 telemetry.addData(">>", "Vuforia initialized, press start to continue...");
//	 telemetry.update();
//   }
//	 
//	 OpenCvCamera camera;
//	 AprilTagDetectionPipeline aprilTagDetectionPipeline;
// 
//	 static final double FEET_PER_METER = 3.28084;
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
//	 int Count;
//	 int BRW_Position;
//	 int BLW_Position;
// 
//	 vuforiaPOWERPLAY = new VuforiaCurrentGame();
//	 FrontRightWheel = hardwareMap.get(DcMotor.class, "Front-Right-Wheel");
//	 BackRightWheel = hardwareMap.get(DcMotor.class, "Back-Right-Wheel");
//	 RotateServo2 = hardwareMap.get(Servo.class, "RotateServo2");
//	 FrontLeftArm = hardwareMap.get(DcMotor.class, "Front-Left-Arm");
//	 GrabServo = hardwareMap.get(Servo.class, "Grab-Servo");
//	 RotateServo1 = hardwareMap.get(Servo.class, "RotateServo1");
//	 CameraServo = hardwareMap.get(Servo.class, "Camera-Servo");
//	 ArmExtendor = hardwareMap.get(DcMotor.class, "Arm-Extendor");
//	 FrontRightArm = hardwareMap.get(DcMotor.class, "Front-Right-Arm");
//	 Stopper = hardwareMap.get(DcMotor.class, "Stopper");
//	 LED_LED = hardwareMap.get(LED.class, "LED");
//	 FrontLeftWheel = hardwareMap.get(DcMotor.class, "Front-Left-Wheel");
//	 BackLeftWheel = hardwareMap.get(DcMotor.class, "Back-Left-Wheel");
//	 imu = hardwareMap.get(BNO055IMU.class, "imu");
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
//	 Initialization();
//	 LeftDownServo.setDirection(Servo.Direction.REVERSE);
//	 LeftDownServo.setPosition(0.375);
//	 sleep(1000);
//	 RightDownServo.setPosition(0.26);
//	 FrontRightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
//	 BackRightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
//	 RotateServo2.setDirection(Servo.Direction.REVERSE);
//	 FrontLeftArm.setDirection(DcMotorSimple.Direction.REVERSE);
//	 GrabServo.setPosition(0);
//	 RotateServo1.setPosition(0.9);
//	 RotateServo2.setPosition(0.9);
//	 CameraServo.setPosition(0.61);
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
//	 Count = 0;
//	 In_Position = false;
//	 Cones_Dropped = 0;
//	 Step = 0;
//	 isVisible = false;
//	 LED_LED.enable(false);
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
//		 /* Actually do something useful */
//		 if(tagOfInterest == null){
//			 telemetry.addLine("Position Unknown: Starting Code!");
//			 telemetry.update();
//		 }else if(tagOfInterest.id == LEFT){
//			 Determined_Location = 1;
//			 telemetry.addLine("Position 1: Starting Code!");
//			 telemetry.update();
//		 }else if(tagOfInterest.id == MIDDLE){
//			 Determined_Location = 2;
//			 telemetry.addLine("Position 2: Starting Code!");
//			 telemetry.update();
//		 }else{
//			 Determined_Location = 3;
//			 telemetry.addLine("Position 3: Starting Code!");
//			 telemetry.update();
//		 }
// 
// 
//		 /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
//		 if (opModeIsActive()) {
//			 vuforiaPOWERPLAY.activate();
//			 vuforiaPOWERPLAY.setActiveCamera(hardwareMap.get(WebcamName.class, "Webcam 2"));
//	   Drive_System(0, 500, 0.6, 0, 1);
//	   ArmExtendor.setTargetPosition(2850);
//	   ArmExtendor.setPower(0.5);
//	   ArmExtendor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	   FrontRightArm.setTargetPosition(2000);
//	   FrontLeftArm.setTargetPosition(2000);
//	   FrontRightArm.setPower(0.75);
//	   FrontLeftArm.setPower(0.75);
//	   FrontRightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	   FrontLeftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	   Drive_System(0, 700, 0.6, 0, 1);
//	   Drive_System(4, 900, 0.4, 0, -1);
//	   FrontRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//	   BackRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//	   FrontLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//	   BackLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//	   BackLeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
//	   BackRightWheel.setDirection(DcMotorSimple.Direction.FORWARD);
//	   FrontLeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
//	   FrontRightWheel.setDirection(DcMotorSimple.Direction.FORWARD);
//	   while (Step == 0) {
//		 // Are the targets visible?
//		 // (Note we only process first visible target).
//		 vuforiaResults = vuforiaPOWERPLAY.track("Blue Audience Wall");
//		 if (vuforiaResults.isVisible) {
//		   isVisible = true;
//		   LED_LED.enable(true);
//		   processTarget();
//		   VuforiaCorrectionCodeX(-38, -400);
//		 } else {
//		   isVisible = false;
//		   LED_LED.enable(false);
//		   telemetry.addData("No Targets Detected", "Targets are not visible.");
//		   FrontRightWheel.setPower(0.1);
//		   BackRightWheel.setPower(0.1);
//		   FrontLeftWheel.setPower(0.1);
//		   BackLeftWheel.setPower(0.1);
//		 }
//		 telemetry.update();
//	   }
//	   Drive_System(3, 800, 0.6, 0, -1);
//	   CameraServo.setPosition(0.7);
//	   Step = 2;
//	   FrontRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//	   BackRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//	   FrontLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//	   BackLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//	   BackLeftWheel.setDirection(DcMotorSimple.Direction.FORWARD);
//	   BackRightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
//	   FrontLeftWheel.setDirection(DcMotorSimple.Direction.FORWARD);
//	   FrontRightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
//	   while (Step == 2) {
//		 // Are the targets visible?
//		 // (Note we only process first visible target).
//		 vuforiaResults = vuforiaPOWERPLAY.track("Blue Audience Wall");
//		 if (vuforiaResults.isVisible) {
//		   isVisible = true;
//		   LED_LED.enable(true);
//		   processTarget();
//		   VuforiaCorrectionCode(-48.3, 4.6, 142, 400, 300, 250);
//		 } else {
//		   isVisible = false;
//		   LED_LED.enable(false);
//		   telemetry.addData("No Targets Detected", "Targets are not visible.");
//		   FrontRightWheel.setPower(0);
//		   BackRightWheel.setPower(0);
//		   FrontLeftWheel.setPower(0);
//		   BackLeftWheel.setPower(0);
//		 }
//		 telemetry.update();
//	   }
//	   In_Position = true;
//	   RightDownServo.setPosition(0.05);
//	   sleep(500);
//	   LeftDownServo.setPosition(0.125);
//	   CameraServo.setPosition(0.61);
//	   BRW_Position = BackRightWheel.getCurrentPosition();
//	   BLW_Position = BackLeftWheel.getCurrentPosition();
//	   BackRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//	   BackLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//	   BackRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	   BackLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	   ArmExtendor.setTargetPosition(2850);
//	   CameraServo.setPosition(0.61);
//	   Stopper.setTargetPosition(120);
//	   Stopper.setPower(1);
//	   Stopper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	   FrontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//	   BackRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//	   FrontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//	   BackLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//	   sleep(1000);
//	   // Pre-Load
//	   Arm_Up();
//	   Drop_Cone();
//	   // Cone 1
//	   Hover_Over_Cone();
//	   Collect_Cone();
//	   Arm_Up();
//	   Drop_Cone();
//	   // Cone 2
//	   Hover_Over_Cone();
//	   Collect_Cone();
//	   Arm_Up();
//	   Drop_Cone();
//	   // Cone 3
//	   Hover_Over_Cone();
//	   Collect_Cone();
//	   Arm_Up();
//	   Drop_Cone();
//	   // Cone 4
//	   Hover_Over_Cone();
//	   Collect_Cone();
//	   Arm_Up();
//	   Drop_Cone();
//	   // Cone 5
//	   Hover_Over_Cone();
//	   Collect_Cone();
//	   Arm_Up();
//	   Drop_Cone();
//	   // Done
//	   Reset();
//	   while (opModeIsActive()) {
//		 // Put loop blocks here.
//		 telemetry.addData("Arm Extension", ArmExtendor.getCurrentPosition());
//		 telemetry.update();
//	   }
//	   // Put run blocks here.
//	 }
//	 vuforiaPOWERPLAY.deactivate();
// 
//	 vuforiaPOWERPLAY.close();
//   }
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
//   /**
//	* Describe this function...
//	*/
//   private void Drive_System(int Mode, int Target_Position, double Speed, int Angle, int Polarity) 
//   {
//	 double Integral;
//	 double Derivative;
//	 double Error2;
//	 double Last_Error;
//	 float Current_Angle;
//	 int temp;
//	 double integral_for_turning;
//	 List integral_list_for_turning;
//	 List _7BlistVariable_7D;
//	 double j;
//	 double k;
//	 // TODO: Enter the type for variable named i
//	 double i;
// 
//	 FrontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	 BackRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	 FrontLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	 BackLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	 if (Polarity == -1) {
//	   BackLeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
//	   BackRightWheel.setDirection(DcMotorSimple.Direction.FORWARD);
//	   FrontLeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
//	   FrontRightWheel.setDirection(DcMotorSimple.Direction.FORWARD);
//	 } else {
//	   BackLeftWheel.setDirection(DcMotorSimple.Direction.FORWARD);
//	   BackRightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
//	   FrontLeftWheel.setDirection(DcMotorSimple.Direction.FORWARD);
//	   FrontRightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
//	 }
//	 if (Mode == 0) {
//	   telemetry.addData("Running", "Forwards");
//	   Derivative = 0;
//	   Integral = 0;
//	   Last_Error = 0;
//	   FrontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	   BackRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	   FrontLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	   BackLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	   FrontRightWheel.setTargetPosition(Target_Position);
//	   BackRightWheel.setTargetPosition(Target_Position);
//	   FrontLeftWheel.setTargetPosition(Target_Position);
//	   BackLeftWheel.setTargetPosition(Target_Position);
//	   FrontRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	   BackRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	   FrontLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	   BackLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	   while (FrontLeftWheel.getCurrentPosition() < Target_Position) {
//		 Error2 = FrontLeftWheel.getCurrentPosition() - FrontRightWheel.getCurrentPosition();
//		 Derivative = Error2 - Last_Error;
//		 Integral = Integral + Error2;
//		 FrontLeftWheel.setPower(Speed);
//		 BackLeftWheel.setPower(Speed);
//		 FrontRightWheel.setPower(FrontLeftWheel.getPower() + Error2 * 0.005 + Integral * 0 + Derivative * 0.001);
//		 BackRightWheel.setPower(FrontLeftWheel.getPower() + Error2 * 0.005 + Integral * 0 + Derivative * 0.001);
//		 Last_Error = Error2;
//		 telemetry.addData("Error", Error2);
//		 telemetry.addData("Derivative", Derivative);
//		 telemetry.addData("Integral", Integral);
//		 telemetry.addData("Determined Location", Determined_Location);
//		 telemetry.update();
//	   }
//	 } else if (Mode == 1) {
//	   telemetry.addData("Running", "Backwards");
//	   Derivative = 0;
//	   Integral = 0;
//	   Last_Error = 0;
//	   FrontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	   BackRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	   FrontLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	   BackLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	   while (FrontLeftWheel.getCurrentPosition() < Target_Position) {
//		 Error2 = FrontRightWheel.getCurrentPosition() - FrontLeftWheel.getCurrentPosition();
//		 Derivative = Error2 - Last_Error;
//		 Integral = Integral + Error2;
//		 FrontLeftWheel.setPower(Speed);
//		 BackLeftWheel.setPower(Speed);
//		 FrontRightWheel.setPower(FrontLeftWheel.getPower() + Error2 * 0.005 + Integral * 0 + Derivative * 0.001);
//		 BackRightWheel.setPower(FrontLeftWheel.getPower() + Error2 * 0.005 + Integral * 0 + Derivative * 0.001);
//		 Last_Error = Error2;
//		 telemetry.addData("Error", Error2);
//		 telemetry.addData("Derivative", Derivative);
//		 telemetry.addData("Integral", Integral);
//		 telemetry.addData("BL", FrontRightWheel.getPower());
//		 telemetry.addData("FL", BackRightWheel.getPower());
//		 telemetry.addData("BR", FrontLeftWheel.getPower());
//		 telemetry.addData("FR", BackLeftWheel.getPower());
//		 telemetry.update();
//	   }
//	 } else if (Mode == 3) {
//	   telemetry.addData("Running", "Strafing");
//	   Derivative = 0;
//	   Integral = 0;
//	   Last_Error = 0;
//	   FrontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	   BackRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	   FrontLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	   BackLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	   FrontRightWheel.setTargetPosition(Target_Position);
//	   BackRightWheel.setTargetPosition(-Target_Position);
//	   FrontLeftWheel.setTargetPosition(-Target_Position);
//	   BackLeftWheel.setTargetPosition(Target_Position);
//	   FrontRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	   BackRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	   FrontLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	   BackLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	   while (BackLeftWheel.getCurrentPosition() < Target_Position) {
//		 Error2 = FrontRightWheel.getCurrentPosition() - BackLeftWheel.getCurrentPosition();
//		 Derivative = Error2 - Last_Error;
//		 Integral = Integral + Error2;
//		 FrontLeftWheel.setPower(Speed);
//		 BackLeftWheel.setPower(Speed);
//		 FrontRightWheel.setPower(FrontLeftWheel.getPower() + Error2 * 0.002 + Integral * 0 + Derivative * 0.001);
//		 BackRightWheel.setPower(FrontLeftWheel.getPower() + Error2 * 0.002 + Integral * 0 + Derivative * 0.001);
//		 Last_Error = Error2;
//		 telemetry.addData("Error", Error2);
//		 telemetry.addData("Derivative", Derivative);
//		 telemetry.addData("Integral", Integral);
//		 telemetry.addData("BL", FrontRightWheel.getPower());
//		 telemetry.addData("FL", BackRightWheel.getPower());
//		 telemetry.addData("BR", FrontLeftWheel.getPower());
//		 telemetry.addData("FR", BackLeftWheel.getPower());
//		 telemetry.update();
//	   }
//	 } else if (Mode == 4) {
//	   telemetry.addData("Running", "Turning");
//	   Derivative = 0;
//	   Integral = 0;
//	   Last_Error = 0;
//	   FrontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	   BackRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	   FrontLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	   BackLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	   FrontRightWheel.setTargetPosition(Target_Position);
//	   BackRightWheel.setTargetPosition(Target_Position);
//	   FrontLeftWheel.setTargetPosition(-Target_Position);
//	   BackLeftWheel.setTargetPosition(-Target_Position);
//	   FrontRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	   BackRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	   FrontLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	   BackLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	   while (Math.abs(FrontRightWheel.getCurrentPosition()) < Target_Position) {
//		 Error2 = FrontRightWheel.getCurrentPosition() - BackRightWheel.getCurrentPosition();
//		 Derivative = Error2 - Last_Error;
//		 Integral = Integral + Error2;
//		 FrontLeftWheel.setPower(Speed);
//		 BackLeftWheel.setPower(Speed);
//		 FrontRightWheel.setPower(FrontLeftWheel.getPower() + Error2 * 0.002 + Integral * 0 + Derivative * 0.001);
//		 BackRightWheel.setPower(FrontLeftWheel.getPower() + Error2 * 0.002 + Integral * 0 + Derivative * 0.001);
//		 Last_Error = Error2;
//		 telemetry.addData("Error", Error2);
//		 telemetry.addData("Derivative", Derivative);
//		 telemetry.addData("Integral", Integral);
//		 telemetry.addData("BL", FrontRightWheel.getPower());
//		 telemetry.addData("FL", BackRightWheel.getPower());
//		 telemetry.addData("BR", FrontLeftWheel.getPower());
//		 telemetry.addData("FR", BackLeftWheel.getPower());
//		 telemetry.update();
//	   }
//	 }
//	 if (Mode == 5) {
//	   telemetry.addData("Running", "Strafing");
//	   Derivative = 0;
//	   Integral = 0;
//	   Last_Error = 0;
//	   FrontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	   BackRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	   FrontLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	   BackLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	   FrontRightWheel.setTargetPosition(-Target_Position);
//	   BackRightWheel.setTargetPosition(-Target_Position);
//	   FrontLeftWheel.setTargetPosition(Target_Position);
//	   BackLeftWheel.setTargetPosition(Target_Position);
//	   FrontRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	   BackRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	   FrontLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	   BackLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	   while (BackLeftWheel.getCurrentPosition() < Target_Position) {
//		 Error2 = FrontRightWheel.getCurrentPosition() - BackLeftWheel.getCurrentPosition();
//		 Derivative = Error2 - Last_Error;
//		 Integral = Integral + Error2;
//		 FrontLeftWheel.setPower(Speed);
//		 BackLeftWheel.setPower(Speed);
//		 FrontRightWheel.setPower(FrontLeftWheel.getPower() + Error2 * 0.002 + Integral * 0 + Derivative * 0.001);
//		 BackRightWheel.setPower(FrontLeftWheel.getPower() + Error2 * 0.002 + Integral * 0 + Derivative * 0.001);
//		 Last_Error = Error2;
//		 telemetry.addData("Error", Error2);
//		 telemetry.addData("Derivative", Derivative);
//		 telemetry.addData("Integral", Integral);
//		 telemetry.addData("BL", FrontRightWheel.getPower());
//		 telemetry.addData("FL", BackRightWheel.getPower());
//		 telemetry.addData("BR", FrontLeftWheel.getPower());
//		 telemetry.addData("FR", BackLeftWheel.getPower());
//		 telemetry.update();
//	   }
//	 }
//   }
// 
//   /**
//	* Describe this function...
//	*/
//   private void VuforiaCorrectionCode(double X_Wanted, double Y_Wanted, double Z_Angle_Wanted, int X_PID_Co, int Y_PID_Co, int Z_Angle_PID_Co) {
//	 double Last_Position_Y;
// 
//	 if (!(Math.abs(X_Wanted - displayValue(vuforiaResults.x, "IN")) < 0.2 && Math.abs(displayValue(vuforiaResults.y, "IN") - Y_Wanted) < 0.2 && Math.abs(vuforiaResults.zAngle - Z_Angle_Wanted) < 0.25)) {
//	   Last_Position_Y = displayValue(vuforiaResults.y, "IN");
//	   Last_Position_X = displayValue(vuforiaResults.x, "IN");
//	   ((DcMotorEx) FrontRightWheel).setVelocity(Math.min(Math.max((X_Wanted - displayValue(vuforiaResults.x, "IN")) * X_PID_Co + -((displayValue(vuforiaResults.y, "IN") - Y_Wanted) * Y_PID_Co) + (vuforiaResults.zAngle - Z_Angle_Wanted) * Z_Angle_PID_Co, -400), 400));
//	   ((DcMotorEx) BackRightWheel).setVelocity(Math.min(Math.max((X_Wanted - displayValue(vuforiaResults.x, "IN")) * X_PID_Co + (displayValue(vuforiaResults.y, "IN") - Y_Wanted) * Y_PID_Co + (vuforiaResults.zAngle - Z_Angle_Wanted) * Z_Angle_PID_Co, -400), 400));
//	   ((DcMotorEx) FrontLeftWheel).setVelocity(Math.min(Math.max((X_Wanted - displayValue(vuforiaResults.x, "IN")) * X_PID_Co + (displayValue(vuforiaResults.y, "IN") - Y_Wanted) * Y_PID_Co + -((vuforiaResults.zAngle - Z_Angle_Wanted) * Z_Angle_PID_Co), -400), 400));
//	   ((DcMotorEx) BackLeftWheel).setVelocity(Math.min(Math.max((X_Wanted - displayValue(vuforiaResults.x, "IN")) * X_PID_Co + -((displayValue(vuforiaResults.y, "IN") - Y_Wanted) * Y_PID_Co) + -((vuforiaResults.zAngle - Z_Angle_Wanted) * Z_Angle_PID_Co), -400), 400));
//	   telemetry.addData("Velocity Y", displayValue(vuforiaResults.y, "IN") - Last_Position_Y);
//	   telemetry.addData("Velocity Y2", imu.getVelocity().yVeloc);
//	   telemetry.addData("Velocity X", displayValue(vuforiaResults.x, "IN") - Last_Position_X);
//	   telemetry.addData("Velocity X2", imu.getVelocity().xVeloc);
//	   telemetry.addData("X-Displacement", Math.abs(X_Wanted - displayValue(vuforiaResults.x, "IN")));
//	   telemetry.addData("Y-Displacement", Math.abs(displayValue(vuforiaResults.y, "IN") - Y_Wanted));
//	   telemetry.addData("Z-Angle-Displacement", Math.abs(vuforiaResults.zAngle - Z_Angle_Wanted));
//	   telemetry.addData("X-Command", (X_Wanted - displayValue(vuforiaResults.x, "IN")) * X_PID_Co);
//	   telemetry.addData("Y-Command", (displayValue(vuforiaResults.y, "IN") - Y_Wanted) * Y_PID_Co);
//	   telemetry.addData("Z-Command", (Z_Angle_Wanted - vuforiaResults.zAngle) * Z_Angle_PID_Co);
//	   telemetry.addData("Location", Determined_Location);
//	   telemetry.update();
//	 } else {
//	   FrontRightWheel.setPower(0);
//	   BackRightWheel.setPower(0);
//	   FrontLeftWheel.setPower(0);
//	   BackLeftWheel.setPower(0);
//	   Step += 1;
//	 }
//   }
// 
//   /**
//	* Describe this function...
//	*/
//   private void Collect_Cone() {
//	 if (Cones_Dropped == 1) {
//	   FrontRightArm.setTargetPosition(500 - Cones_Dropped * 50);
//	   FrontLeftArm.setTargetPosition(500 - Cones_Dropped * 50);
//	   FrontRightArm.setPower(0.3);
//	   FrontLeftArm.setPower(0.3);
//	 } else if (Cones_Dropped == 2) {
//	   FrontRightArm.setTargetPosition(500 - Cones_Dropped * 50);
//	   FrontLeftArm.setTargetPosition(500 - Cones_Dropped * 50);
//	   FrontRightArm.setPower(0.3);
//	   FrontLeftArm.setPower(0.3);
//	 } else if (Cones_Dropped == 3) {
//	   FrontRightArm.setTargetPosition(500 - Cones_Dropped * 50);
//	   FrontLeftArm.setTargetPosition(500 - Cones_Dropped * 50);
//	   FrontRightArm.setPower(0.3);
//	   FrontLeftArm.setPower(0.3);
//	 } else if (Cones_Dropped == 4) {
//	   FrontRightArm.setTargetPosition(500 - Cones_Dropped * 50);
//	   FrontLeftArm.setTargetPosition(500 - Cones_Dropped * 50);
//	   FrontRightArm.setPower(0.3);
//	   FrontLeftArm.setPower(0.3);
//	 } else if (Cones_Dropped == 5) {
//	   FrontRightArm.setTargetPosition(500 - Cones_Dropped * 50);
//	   FrontLeftArm.setTargetPosition(500 - Cones_Dropped * 50);
//	   FrontRightArm.setPower(0.3);
//	   FrontLeftArm.setPower(0.3);
//	 }
//	 sleep(100);
//	 if (Cones_Dropped == 1) {
//	   RotateServo1.setPosition(0.05);
//	   RotateServo2.setPosition(0.05);
//	 } else if (Cones_Dropped == 2) {
//	   RotateServo1.setPosition(0.07);
//	   RotateServo2.setPosition(0.07);
//	 } else if (Cones_Dropped == 3) {
//	   RotateServo1.setPosition(0.09);
//	   RotateServo2.setPosition(0.09);
//	 } else if (Cones_Dropped == 4) {
//	   RotateServo1.setPosition(0.11);
//	   RotateServo2.setPosition(0.11);
//	 } else if (Cones_Dropped == 5) {
//	   RotateServo1.setPosition(0.13);
//	   RotateServo2.setPosition(0.13);
//	 }
//	 sleep(1000);
//   }
// 
//   /**
//	* Describe this function...
//	*/
//   private double displayValue(float originalValue, String units) {
//	 double convertedValue;
// 
//	 // Vuforia returns distances in mm.
//	 if (units.equals("CM")) {
//	   convertedValue = originalValue / 10;
//	 } else if (units.equals("M")) {
//	   convertedValue = originalValue / 1000;
//	 } else if (units.equals("IN")) {
//	   convertedValue = originalValue / 25.4;
//	 } else if (units.equals("FT")) {
//	   convertedValue = (originalValue / 25.4) / 12;
//	 } else {
//	   convertedValue = originalValue;
//	 }
//	 return convertedValue;
//   }
// 
//   /**
//	* Describe this function...
//	*/
//   private void processTarget() {
//	 // Display the target name.
//	 telemetry.addData("Target Detected", vuforiaResults.name + " is visible.");
//	 telemetry.addData("X (in)", Double.parseDouble(JavaUtil.formatNumber(displayValue(vuforiaResults.x, "IN"), 2)));
//	 telemetry.addData("Y (in)", Double.parseDouble(JavaUtil.formatNumber(displayValue(vuforiaResults.y, "IN"), 2)));
//	 telemetry.addData("Z (in)", Double.parseDouble(JavaUtil.formatNumber(displayValue(vuforiaResults.z, "IN"), 2)));
//	 telemetry.addData("Rotation about Z (deg)", Double.parseDouble(JavaUtil.formatNumber(vuforiaResults.zAngle, 2)));
//   }
// 
//   /**
//	* Describe this function...
//	*/
//   private void VuforiaCorrectionCodeX(int X_Wanted, int X_PID_Co) {
//	 if (!(Math.abs(X_Wanted - displayValue(vuforiaResults.x, "IN")) < 0.2)) {
//	   Last_Position_X = displayValue(vuforiaResults.x, "IN");
//	   ((DcMotorEx) FrontRightWheel).setVelocity(Math.min(Math.max((X_Wanted - displayValue(vuforiaResults.x, "IN")) * X_PID_Co, -300), 300));
//	   ((DcMotorEx) BackRightWheel).setVelocity(Math.min(Math.max((X_Wanted - displayValue(vuforiaResults.x, "IN")) * X_PID_Co, -300), 300));
//	   ((DcMotorEx) FrontLeftWheel).setVelocity(Math.min(Math.max((X_Wanted - displayValue(vuforiaResults.x, "IN")) * X_PID_Co, -300), 300));
//	   ((DcMotorEx) BackLeftWheel).setVelocity(Math.min(Math.max((X_Wanted - displayValue(vuforiaResults.x, "IN")) * X_PID_Co, -300), 300));
//	   telemetry.addData("Velocity X", displayValue(vuforiaResults.x, "IN") - Last_Position_X);
//	   telemetry.addData("X-Displacement", Math.abs(X_Wanted - displayValue(vuforiaResults.x, "IN")));
//	   telemetry.addData("X-Command", (X_Wanted - displayValue(vuforiaResults.x, "IN")) * X_PID_Co);
//	   telemetry.addData("Location", Determined_Location);
//	   telemetry.update();
//	 } else {
//	   FrontRightWheel.setPower(0);
//	   BackRightWheel.setPower(0);
//	   FrontLeftWheel.setPower(0);
//	   BackLeftWheel.setPower(0);
//	   Step += 1;
//	 }
//   }
// 
//   /**
//	* Describe this function...
//	*/
//   private void Arm_Up() {
//	 FrontRightArm.setTargetPosition(2550);
//	 FrontLeftArm.setTargetPosition(2550);
//	 FrontRightArm.setPower(0.75);
//	 FrontLeftArm.setPower(0.75);
//	 sleep(200);
//	 if (Cones_Dropped != 0) {
//	   RotateServo2.setPosition(0);
//	   RotateServo1.setPosition(0);
//	 }
//	 sleep(200);
//	 RotateServo1.setPosition(0.85);
//	 RotateServo2.setPosition(0.85);
//	 if (Cones_Dropped == 0) {
//	   sleep(750);
//	 } else {
//	   sleep(1000);
//	 }
//   }
// 
//   /**
//	* Describe this function...
//	*/
//   private void Hover_Over_Cone() {
//	 RotateServo1.setPosition(0);
//	 RotateServo2.setPosition(0);
//	 FrontRightArm.setTargetPosition(700);
//	 FrontLeftArm.setTargetPosition(700);
//	 FrontRightArm.setPower(0.6);
//	 FrontLeftArm.setPower(0.6);
//	 FrontRightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	 FrontLeftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	 sleep(1500);
//   }
// 
//   /**
//	* Describe this function...
//	*/
//   private void Reset() {
//	 GrabServo.setPosition(0);
//	 FrontRightArm.setTargetPosition(0);
//	 FrontLeftArm.setTargetPosition(0);
//	 FrontRightArm.setPower(0.5);
//	 FrontLeftArm.setPower(0.5);
//	 RotateServo1.setPosition(0.875);
//	 RotateServo2.setPosition(0.875);
//	 ArmExtendor.setTargetPosition(0);
//	 ArmExtendor.setPower(1);
//	 Stopper.setTargetPosition(0);
//	 Stopper.setPower(1);
//	 Stopper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	 In_Position = true;
//   }
// 
//   /**
//	* Describe this function...
//	*/
//   private void Drop_Cone() {
//	 GrabServo.setPosition(0.4);
//	 Cones_Dropped += 1;
//	 sleep(500);
//	 GrabServo.setPosition(0);
//   }
// }
