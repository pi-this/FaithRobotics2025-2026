// /*package org.firstinspires.ftc.teamcode.old;
// 
// import com.qualcomm.hardware.bosch.BNO055IMU;
// import java.util.Scanner;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.DcMotorSimple;
// import java.util.List;
// import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
// import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
// import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
// import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
// import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
// import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
// import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
// import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
// import org.firstinspires.ftc.robotcore.external.tfod.TfodCurrentGame;
// 
// @TeleOp(name = "AutoX (Blocks to Java)")
// public class AutoX extends LinearOpMode {
// 
//   private DcMotor FL0;
//   private DcMotor FR1;
//   private DcMotor RR2;
//   private DcMotor RL3;
//   private TfodCurrentGame tfodUltimateGoal;
//   private BNO055IMU imu;
//   private VuforiaCurrentGame vuforiaUltimateGoal;
// 
//   String word_count;
//   Recognition recognition;
//   float Z_Angle;
//   double loopy_counts;
// 
//   /**
//	* This function is executed when this Op Mode is selected from the Driver Station.
//	*/
//   /*@Override
//   public void runOpMode() {
//	 FL0 = hardwareMap.get(DcMotor.class, "FL0");
//	 FR1 = hardwareMap.get(DcMotor.class, "FR1");
//	 RR2 = hardwareMap.get(DcMotor.class, "RR2");
//	 RL3 = hardwareMap.get(DcMotor.class, "RL3");
//	 tfodUltimateGoal = new TfodCurrentGame();
//	 imu = hardwareMap.get(BNO055IMU.class, "imu");
//	 vuforiaUltimateGoal = new VuforiaCurrentGame();
// 
//	 // Sets all Motor to Forword
//	 FL0.setDirection(DcMotorSimple.Direction.FORWARD);
//	 FR1.setDirection(DcMotorSimple.Direction.REVERSE);
//	 RR2.setDirection(DcMotorSimple.Direction.REVERSE);
//	 RL3.setDirection(DcMotorSimple.Direction.FORWARD);
//	 FL0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	 FR1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	 RR2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	 RL3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	 telemetry.addData("Int Cam and IMU", "wait");
//	 telemetry.update();
//	 // Wait for start command from Driver Station.
//	 Cam_Int();
//	 loopy_counts = 0;
//	 word_count = 0;
//	 telemetry.update();
//	 while (!opModeIsActive()) {
//	   loopy_counts += 1;
//	   telemetry.addData("Loop Cycle", loopy_counts);
//	   IMU_Scanner();
//	   Scanner();
//	 }
//	 waitForStart();
//	 tfodUltimateGoal.deactivate();
//	 if (opModeIsActive()) {
//	   // Put run blocks here.
//	   IMU_Drive(3000, 3, 0.4, 90, 1);
//	   IMU_Drive(-3000, 3, 0.4, 0, 1);
//	 }
//	 // Deactivate TFOD.
// 
//	 tfodUltimateGoal.close();
//	 vuforiaUltimateGoal.close();
//   }
// 
//   /**
//	* Describe this function...
//	*/
//   /*private void Cam_Int() {
//	 BNO055IMU.Parameters IMU_Parameters;
// 
//	 // Sample TFOD Op Mode
//	 IMU_Parameters = new BNO055IMU.Parameters();
//	 IMU_Parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//	 IMU_Parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//	 IMU_Parameters.loggingEnabled = false;
//	 imu.initialize(IMU_Parameters);
//	 // Initialize Vuforia.
//	 vuforiaUltimateGoal.initialize(
//		 "", // vuforiaLicenseKey
//		 hardwareMap.get(WebcamName.class, "Webcam 1"), // cameraName
//		 "", // webcamCalibrationFilename
//		 false, // useExtendedTracking
//		 false, // enableCameraMonitoring
//		 VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES, // cameraMonitorFeedback
//		 0, // dx
//		 0, // dy
//		 0, // dz
//		 0, // xAngle
//		 0, // yAngle
//		 0, // zAngle
//		 true); // useCompetitionFieldTargetLocations
//	 // Set min confidence threshold to 0.7
//	 tfodUltimateGoal.initialize(vuforiaUltimateGoal, 0.7F, true, true);
//	 // Initialize TFOD before waitForStart.
//	 // Init TFOD here so the object detection labels are visible
//	 // in the Camera Stream preview window on the Driver Station.
//	 tfodUltimateGoal.activate();
//	 // Enable following block to zoom in on target.
//	 tfodUltimateGoal.setZoom(2.5, 16 / 9);
//	 telemetry.addData(">", "Press Play to start");
//   }
// 
//   /**
//	* Describe this function...
//	*/
//   /*private void Scanner() {
//	 List<Recognition> recognitions;
//	 double index;
// 
//	 // Put loop blocks here.
//	 // Get a list of recognitions from TFOD.
//	 recognitions = tfodUltimateGoal.getRecognitions();
//	 // If list is empty, inform the user. Otherwise, go
//	 // through list and display info for each recognition.
//	 if (recognitions.size() == 0) {
//	   telemetry.addData("TFOD", "No items detected.");
//	 } else {
//	   index = 0;
//	   // Iterate through list and call a function to
//	   // display info for each recognized object.
//	   for (Recognition recognition_item : recognitions) {
//		 recognition = recognition_item;
//		 // Display info.
//		 displayInfo(index);
//		 // Increment index.
//		 index = index + 1;
//	   }
//	 }
//	 telemetry.update();
//   }
// 
//   /**
//	* Display info (using telemetry) for a recognized object.
//	*/
//   /*private void displayInfo(double i) {
//	 word_count = recognition.getLabel();
//	 // Display label info.
//	 // Display the label and index number for the recognition.
//	 telemetry.addData("label " + i, recognition.getLabel());
//	 // Display upper corner info.
//	 // Display the location of the top left corner
//	 // of the detection boundary for the recognition
//	 telemetry.addData("Left, Top " + i, recognition.getLeft() + ", " + recognition.getTop());
//	 // Display lower corner info.
//	 // Display the location of the bottom right corner
//	 // of the detection boundary for the recognition
//	 telemetry.addData("Right, Bottom " + i, recognition.getRight() + ", " + recognition.getBottom());
//	 if (word_count.length() == 6) {
//	   telemetry.addData("single", "100");
//	 } else if (word_count.length() == 4) {
//	   telemetry.addData("quad", "400");
//	 } else {
//	   telemetry.addData("no find???!!!", "X");
//	 }
//   }
// 
//   /**
//	* Describe this function...
//	*/
//   /*private void IMU_Scanner() {
//	 Orientation Angles;
//	 Acceleration Gravity;
//	 float Y_Angle;
//	 float X_Angle;
// 
//	 Angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//	 Gravity = imu.getGravity();
//	 Z_Angle = Angles.firstAngle;
//	 Y_Angle = Angles.secondAngle;
//	 X_Angle = Angles.thirdAngle;
//	 telemetry.addData("rot about Z", Angles.firstAngle);
//	 telemetry.addData("rot about Y", Angles.secondAngle);
//	 telemetry.addData("rot about X", Angles.thirdAngle);
//	 telemetry.addData("gravity (Z)", Gravity.zAccel);
//	 telemetry.addData("gravity (Y)", Gravity.yAccel);
//	 telemetry.addData("gravity (Z)", Gravity.xAccel);
//   }
// 
//   /**
//	* Describe this function...
//	*/
//   /*private void Basic_Tick_Drive(int Input_Ticks, double Function, double Input_Speed) {
//	 if (1 == Function) {
//	   FL0.setPower(Input_Speed);
//	   FR1.setPower(Input_Speed);
//	   RR2.setPower(Input_Speed);
//	   RL3.setPower(Input_Speed);
//	   FL0.setTargetPosition(Input_Ticks);
//	   FR1.setTargetPosition(Input_Ticks);
//	   RR2.setTargetPosition(Input_Ticks);
//	   RL3.setTargetPosition(Input_Ticks);
//	 } else if (2 == Function) {
//	   FL0.setPower(Input_Speed);
//	   FR1.setPower(-Input_Speed);
//	   RR2.setPower(Input_Speed);
//	   RL3.setPower(-Input_Speed);
//	   FL0.setTargetPosition(Input_Ticks);
//	   FR1.setTargetPosition(-Input_Ticks);
//	   RR2.setTargetPosition(Input_Ticks);
//	   RL3.setTargetPosition(-Input_Ticks);
//	 } else if (3 == Function) {
//	   FL0.setPower(Input_Speed);
//	   FR1.setPower(Input_Speed);
//	   RR2.setPower(Input_Speed);
//	   RL3.setPower(Input_Speed);
//	   FL0.setTargetPosition(Input_Ticks);
//	   FR1.setTargetPosition(-Input_Ticks);
//	   RR2.setTargetPosition(-Input_Ticks);
//	   RL3.setTargetPosition(Input_Ticks);
//	 }
//	 FL0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	 FR1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	 RR2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	 RL3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	 FL0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	 FR1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	 RR2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	 RL3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//   }
// 
//   /**
//	* Describe this function...
//	*/
//   /*private void IMU_Drive(int Input_Ticks, double Function, double Input_Speed, double Angle_Want, double IMU_Polarity) {
//	 double Angle_Input;
// 
//	 FL0.setPower(0);
//	 FR1.setPower(0);
//	 RR2.setPower(0);
//	 RL3.setPower(0);
//	 if (1 == Function) {
//	   FL0.setTargetPosition(Input_Ticks);
//	   FR1.setTargetPosition(Input_Ticks);
//	   RR2.setTargetPosition(Input_Ticks);
//	   RL3.setTargetPosition(Input_Ticks);
//	 } else if (3 == Function) {
//	   FL0.setTargetPosition(Input_Ticks);
//	   FR1.setTargetPosition(Input_Ticks);
//	   RR2.setTargetPosition(Input_Ticks);
//	   RL3.setTargetPosition(Input_Ticks);
//	 }
//	 FL0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	 FR1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	 RR2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	 RL3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	 if (1 == Function) {
//	   FL0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	   FR1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	   RR2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	   RL3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	 } else if (3 == Function) {
//	   FL0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//	   FR1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//	   RR2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//	   RL3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//	 }
//	 loopy_counts = 1;
//	 while (opModeIsActive()) {
//	   loopy_counts += 1;
//	   telemetry.addData("Loop Cycle", loopy_counts);
//	   IMU_Scanner();
//	   if (1 == Function) {
//		 Angle_Input = 0.02 * (Angle_Want - Z_Angle);
//	   } else if (3 == Function) {
//		 Angle_Input = (Angle_Want - Z_Angle) / Math.abs(Angle_Want - Z_Angle);
//	   }
//	   if (1 == Function) {
//		 if (Math.abs(FL0.getCurrentPosition()) - 40 <= Input_Ticks && Input_Ticks <= Math.abs(FL0.getCurrentPosition()) + 40) {
//		   break;
//		 }
//		 if (1 == IMU_Polarity) {
//		   FL0.setPower(Input_Speed - Angle_Input);
//		   FR1.setPower(Input_Speed + Angle_Input);
//		   RR2.setPower(Input_Speed + Angle_Input);
//		   RL3.setPower(Input_Speed - Angle_Input);
//		 } else if (2 == IMU_Polarity) {
//		   FL0.setPower(Input_Speed + Angle_Input);
//		   FR1.setPower(Input_Speed - Angle_Input);
//		   RR2.setPower(Input_Speed - Angle_Input);
//		   RL3.setPower(Input_Speed + Angle_Input);
//		 }
//	   } else if (3 == Function) {
//		 if (Angle_Want - 2 <= Z_Angle && Z_Angle <= Angle_Want + 2) {
//		   break;
//		 }
//		 if (1 == IMU_Polarity) {
//		   FL0.setPower(Input_Speed * Angle_Input);
//		   FR1.setPower(-(Input_Speed * Angle_Input));
//		   RR2.setPower(-(Input_Speed + Angle_Input));
//		   RL3.setPower(Input_Speed * Angle_Input);
//		 } else if (2 == IMU_Polarity) {
//		   FL0.setPower(-(Input_Speed * Angle_Input));
//		   FR1.setPower(Input_Speed * Angle_Input);
//		   RR2.setPower(Input_Speed * Angle_Input);
//		   RL3.setPower(-(Input_Speed * Angle_Input));
//		 }
//	   }
//	   telemetry.update();
//	 }
//   }
// }
// */
