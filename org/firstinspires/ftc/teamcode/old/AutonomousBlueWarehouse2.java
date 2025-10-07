// /*package org.firstinspires.ftc.teamcode.old;
// 
// import com.qualcomm.hardware.bosch.BNO055IMU;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.hardware.CRServo;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.DcMotorSimple;
// import com.qualcomm.robotcore.hardware.DistanceSensor;
// import com.qualcomm.robotcore.hardware.Servo;
// import com.qualcomm.robotcore.hardware.TouchSensor;
// import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
// import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
// import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
// import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
// 
// @Autonomous(name = "AutonomousBlueWarehouse2 (Blocks to Java)")
// public class AutonomousBlueWarehouse2 extends LinearOpMode {
// 
//   private DcMotor GobmotorFL2;
//   private DcMotor GobmotorBL3;
//   private Servo ArmRotationX;
//   private DcMotor Elevation;
//   private DcMotor GobmotorFR0;
//   private DcMotor GobmotorBR1;
//   private DcMotor TapeMeasure;
//   private DcMotor extendor;
//   private TouchSensor touchsensor2;
//   private DistanceSensor DistanceSensor1;
//   private DistanceSensor DistanceSensor2;
//   private DcMotor collector;
//   private TouchSensor touchsensor3;
//   private BNO055IMU imu;
//   private CRServo carservo1;
//   private CRServo carservo2;
// 
//   /**
//	* This function is executed when this Op Mode is selected from the Driver Station.
//	*/
//   /*@Override
//   public void runOpMode() {
//	 double Determined_Block_Position;
//	 double Distance_Sensor_Active;
// 
//	 GobmotorFL2 = hardwareMap.get(DcMotor.class, "Gobmotor FL 2");
//	 GobmotorBL3 = hardwareMap.get(DcMotor.class, "Gobmotor BL 3");
//	 ArmRotationX = hardwareMap.get(Servo.class, "Arm Rotation X");
//	 Elevation = hardwareMap.get(DcMotor.class, "Elevation");
//	 GobmotorFR0 = hardwareMap.get(DcMotor.class, "Gobmotor FR 0");
//	 GobmotorBR1 = hardwareMap.get(DcMotor.class, "Gobmotor BR 1");
//	 TapeMeasure = hardwareMap.get(DcMotor.class, "Tape Measure");
//	 extendor = hardwareMap.get(DcMotor.class, "extendor");
//	 touchsensor2 = hardwareMap.get(TouchSensor.class, "touchsensor2");
//	 DistanceSensor1 = hardwareMap.get(DistanceSensor.class, "Distance Sensor 1");
//	 DistanceSensor2 = hardwareMap.get(DistanceSensor.class, "Distance Sensor 2");
//	 collector = hardwareMap.get(DcMotor.class, "collector");
//	 touchsensor3 = hardwareMap.get(TouchSensor.class, "touchsensor3");
//	 imu = hardwareMap.get(BNO055IMU.class, "imu");
//	 carservo1 = hardwareMap.get(CRServo.class, "car. servo1");
//	 carservo2 = hardwareMap.get(CRServo.class, "car. servo2");
// 
//	 GobmotorFL2.setDirection(DcMotorSimple.Direction.REVERSE);
//	 GobmotorBL3.setDirection(DcMotorSimple.Direction.REVERSE);
//	 ArmRotationX.setPosition(0.5715);
//	 Elevation.setPower(0);
//	 GobmotorFR0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	 GobmotorBR1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	 GobmotorFL2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	 GobmotorBL3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	 TapeMeasure.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	 extendor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	 GobmotorFR0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//	 GobmotorBR1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//	 GobmotorFL2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//	 GobmotorBL3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//	 Initialize_IMU();
//	 telemetry.update();
//	 Distance_Sensor_Active = 1;
//	 Determined_Block_Position = 0;
//	 sleep(1000);
//	 if (touchsensor2.isPressed()) {
//	   Elevation.setPower(0);
//	 } else {
//	   Elevation.setPower(-0.4);
//	 }
//	 while (!opModeIsActive()) {
//	   if (DistanceSensor1.getDistance(DistanceUnit.CM) < 80) {
//		 Determined_Block_Position = 2;
//		 Distance_Sensor_Active = 0;
//	   } else if (DistanceSensor2.getDistance(DistanceUnit.CM) < 80) {
//		 Determined_Block_Position = 3;
//		 Distance_Sensor_Active = 0;
//	   } else {
//		 Determined_Block_Position = 1;
//		 Distance_Sensor_Active = 0;
//	   }
//	   if (touchsensor2.isPressed()) {
//		 Elevation.setPower(0);
//		 Elevation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	   } else {
//		 telemetry.addData("", "!WAIT TO START UNTIL BUTTON PRESSED IS TRUE!");
//	   }
//	   telemetry.addData("Distance1", DistanceSensor1.getDistance(DistanceUnit.CM));
//	   telemetry.addData("Distance2", DistanceSensor2.getDistance(DistanceUnit.CM));
//	   telemetry.addData("Button Pressed?", touchsensor2.isPressed());
//	   telemetry.update();
//	 }
//	 waitForStart();
//	 if (opModeIsActive()) {
//	   extendor.setTargetPosition(600);
//	   extendor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	   extendor.setPower(-1);
//	   if (Determined_Block_Position == 1) {
//		 Elevation.setTargetPosition(950);
//		 Elevation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//		 Elevation.setPower(1);
//	   } else if (Determined_Block_Position == 2) {
//		 Elevation.setTargetPosition(1900);
//		 Elevation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//		 Elevation.setPower(1);
//	   } else if (Determined_Block_Position == 3) {
//		 Elevation.setTargetPosition(2750);
//		 Elevation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//		 Elevation.setPower(1);
//	   }
//	   sleep(300);
//	   ArmRotationX.setPosition(0.515);
//	   IMU_Drive_System(-700, 1, -0.5, 0, 1, null);
//	   if (Determined_Block_Position == 1) {
//		 IMU_Drive_System(11000, 2, 0.5, 0, 1, null);
//	   } else if (Determined_Block_Position == 2) {
//	   } else if (Determined_Block_Position == 3) {
//	   }
//	   IMU_Drive_System(0, 3, 0.5, 0, 2, null);
//	   collector.setPower(0.5);
//	   sleep(500);
//	   collector.setPower(0);
//	   if (Determined_Block_Position == 1) {
//		 IMU_Drive_System(11000, 2, 0.5, 0, 2, null);
//	   } else if (Determined_Block_Position == 2) {
//	   } else if (Determined_Block_Position == 3) {
//	   }
//	   ArmRotationX.setPosition(0.574);
//	   IMU_Drive_System(1400, 1, 0.2, 0, 1, null);
//	   Elevation.setTargetPosition(0);
//	   Elevation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	   Elevation.setPower(1);
//	   GobmotorFR0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	   GobmotorBR1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	   GobmotorFL2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	   GobmotorBL3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	   while (!(touchsensor3.isPressed() == true || GobmotorFR0.getCurrentPosition() == 700)) {
//		 collector.setPower(-1);
//		 GobmotorFR0.setTargetPosition(700);
//		 GobmotorBR1.setTargetPosition(700);
//		 GobmotorFL2.setTargetPosition(700);
//		 GobmotorBL3.setTargetPosition(700);
//		 GobmotorFR0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//		 GobmotorBR1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//		 GobmotorFL2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//		 GobmotorBL3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//		 GobmotorFR0.setPower(-0.2);
//		 GobmotorBR1.setPower(0.2);
//		 GobmotorFL2.setPower(-0.2);
//		 GobmotorBL3.setPower(0.2);
//	   }
//	   collector.setPower(0);
//	   ArmRotationX.setPosition(0.51);
//	   Elevation.setTargetPosition(2950);
//	   Elevation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	   Elevation.setPower(1);
//	   IMU_Drive_System(-1600, 1, -0.4, 0, 1, 0);
//	   IMU_Drive_System(19000, 2, 0.5, 0, 1, null);
//	   sleep(20000);
//	 }
//   }
// 
//   /**
//	* Describe this function...
//	*/
//   /*private void Initialize_IMU() {
//	 BNO055IMU.Parameters imuParameters;
// 
//	 // Create new IMU Parameters object.
//	 imuParameters = new BNO055IMU.Parameters();
//	 // Use degrees as angle unit.
//	 imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//	 // Express acceleration as m/s^2.
//	 imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//	 // Disable logging.
//	 imuParameters.loggingEnabled = false;
//	 // Initialize IMU.
//	 imu.initialize(imuParameters);
//	 // Prompt user to press start buton.
//	 telemetry.addData("IMU Example", "Press start to continue...");
//   }
// 
//   /**
//	* Describe this function...
//	*/
//   /*private void IMU_Drive_System(int Input_Ticks, double Drive_Mode, double Input_Speed, double Angle_Want, double IMU_Polarity, double Duration) {
//	 float Z_Angle;
//	 double Loop_Cycle_Counter;
//	 Orientation angles;
//	 double Input_Angle;
//	 double IMU_Slow_Turn;
// 
//	 GobmotorFR0.setPower(0);
//	 GobmotorBR1.setPower(0);
//	 GobmotorFL2.setPower(0);
//	 GobmotorBL3.setPower(0);
//	 if (1 == Drive_Mode) {
//	   GobmotorFR0.setTargetPosition(Input_Ticks);
//	   GobmotorBR1.setTargetPosition(Input_Ticks);
//	   GobmotorFL2.setTargetPosition(Input_Ticks);
//	   GobmotorBL3.setTargetPosition(Input_Ticks);
//	 } else if (2 == Drive_Mode) {
//	   TapeMeasure.setTargetPosition(Input_Ticks);
//	 } else if (3 == Drive_Mode) {
//	   GobmotorFR0.setTargetPosition(Input_Ticks);
//	   GobmotorBR1.setTargetPosition(Input_Ticks);
//	   GobmotorFL2.setTargetPosition(Input_Ticks);
//	   GobmotorBL3.setTargetPosition(Input_Ticks);
//	 }
//	 GobmotorFR0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	 GobmotorBR1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	 GobmotorFL2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	 GobmotorBL3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	 TapeMeasure.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	 if (1 == Drive_Mode) {
//	   GobmotorFR0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	   GobmotorBR1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	   GobmotorFL2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	   GobmotorBL3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	 } else if (2 == Drive_Mode) {
//	   TapeMeasure.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	   GobmotorFR0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//	   GobmotorBR1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//	   GobmotorFL2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//	   GobmotorBL3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//	 } else if (3 == Drive_Mode) {
//	   GobmotorFR0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//	   GobmotorBR1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//	   GobmotorFL2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//	   GobmotorBL3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//	 }
//	 Loop_Cycle_Counter = 1;
//	 while (opModeIsActive()) {
//	   Loop_Cycle_Counter += 1;
//	   telemetry.addData("Loop Cycle", Loop_Cycle_Counter);
//	   telemetry.addData("FR 0", GobmotorFR0.getCurrentPosition());
//	   telemetry.addData("BR 1", GobmotorBR1.getCurrentPosition());
//	   telemetry.addData("RR 2", GobmotorFL2.getCurrentPosition());
//	   telemetry.addData("RL 3", GobmotorBL3.getCurrentPosition());
//	   telemetry.addData("Strafe Ticks", TapeMeasure.getCurrentPosition());
//	   // Get absolute orientation
//	   angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//	   // Display orientation info.
//	   Z_Angle = angles.firstAngle;
//	   telemetry.addData("rot about Z", angles.firstAngle);
//	   telemetry.addData("rot about Y", angles.secondAngle);
//	   telemetry.addData("rot about X", angles.thirdAngle);
//	   telemetry.update();
//	   if (1 == Drive_Mode) {
//		 Input_Angle = 0.02 * (Angle_Want - Z_Angle);
//	   } else if (2 == Drive_Mode) {
//		 Input_Angle = 0.02 * (Angle_Want - Z_Angle);
//	   } else if (3 == Drive_Mode) {
//		 Input_Angle = (Angle_Want - Z_Angle) / Math.abs(Angle_Want - Z_Angle);
//		 if ((Angle_Want - Z_Angle) / Math.abs(Angle_Want - Z_Angle) >= 0) {
//		   IMU_Slow_Turn = Math.min(Math.max(0.02 * (Angle_Want - Z_Angle), 0.15), 1);
//		 } else if ((Angle_Want - Z_Angle) / Math.abs(Angle_Want - Z_Angle) <= 0) {
//		   IMU_Slow_Turn = Math.min(Math.max(0.02 * (Z_Angle - Angle_Want), 0.15), 1);
//		 }
//	   }
//	   if (1 == Drive_Mode) {
//		 if (Math.abs(GobmotorFR0.getCurrentPosition()) - 40 <= Math.abs(Input_Ticks) && Math.abs(Input_Ticks) <= Math.abs(GobmotorFR0.getCurrentPosition()) + 40) {
//		   GobmotorFR0.setPower(0);
//		   GobmotorBR1.setPower(0);
//		   GobmotorFL2.setPower(0);
//		   GobmotorBL3.setPower(0);
//		   break;
//		 }
//		 if (1 == IMU_Polarity) {
//		   GobmotorFR0.setPower(Input_Speed + Input_Angle);
//		   GobmotorBR1.setPower(Input_Speed + Input_Angle);
//		   GobmotorFL2.setPower(Input_Speed - Input_Angle);
//		   GobmotorBL3.setPower(Input_Speed - Input_Angle);
//		 } else if (2 == IMU_Polarity) {
//		   GobmotorFR0.setPower(Input_Speed - Input_Angle);
//		   GobmotorBR1.setPower(Input_Speed - Input_Angle);
//		   GobmotorFL2.setPower(Input_Speed + Input_Angle);
//		   GobmotorBL3.setPower(Input_Speed + Input_Angle);
//		 }
//	   } else if (2 == Drive_Mode) {
//		 if (Math.abs(TapeMeasure.getCurrentPosition()) >= Input_Ticks) {
//		   GobmotorFR0.setPower(0);
//		   GobmotorBR1.setPower(0);
//		   GobmotorFL2.setPower(0);
//		   GobmotorBL3.setPower(0);
//		   break;
//		 }
//		 if (1 == IMU_Polarity) {
//		   // Strafe Left
//		   GobmotorFR0.setPower(-(Input_Speed + Input_Angle));
//		   GobmotorBR1.setPower(Input_Speed + Input_Angle);
//		   GobmotorFL2.setPower(Input_Speed - Input_Angle);
//		   GobmotorBL3.setPower(-(Input_Speed - Input_Angle));
//		 } else if (2 == IMU_Polarity) {
//		   // Strafe Right
//		   GobmotorFR0.setPower(Input_Speed - Input_Angle);
//		   GobmotorBR1.setPower(-(Input_Speed - Input_Angle));
//		   GobmotorFL2.setPower(-(Input_Speed + Input_Angle));
//		   GobmotorBL3.setPower(Input_Speed + Input_Angle);
//		 }
//	   } else if (3 == Drive_Mode) {
//		 if (Angle_Want - 1 <= Z_Angle && Z_Angle <= Angle_Want + 1) {
//		   GobmotorFR0.setPower(0);
//		   GobmotorBR1.setPower(0);
//		   GobmotorFL2.setPower(0);
//		   GobmotorBL3.setPower(0);
//		   break;
//		 }
//		 if (1 == IMU_Polarity) {
//		   GobmotorFR0.setPower(-(Input_Speed * Input_Angle * IMU_Slow_Turn));
//		   GobmotorBR1.setPower(-(Input_Speed * Input_Angle * IMU_Slow_Turn));
//		   GobmotorFL2.setPower(Input_Speed * Input_Angle * IMU_Slow_Turn);
//		   GobmotorBL3.setPower(Input_Speed * Input_Angle * IMU_Slow_Turn);
//		 } else if (2 == IMU_Polarity) {
//		   GobmotorFR0.setPower(Input_Speed * Input_Angle * IMU_Slow_Turn);
//		   GobmotorBR1.setPower(Input_Speed * Input_Angle * IMU_Slow_Turn);
//		   GobmotorFL2.setPower(-(Input_Speed * Input_Angle * IMU_Slow_Turn));
//		   GobmotorBL3.setPower(-(Input_Speed * Input_Angle * IMU_Slow_Turn));
//		 }
//	   } else if (5 == Drive_Mode) {
//		 if (1 == IMU_Polarity) {
//		   GobmotorFR0.setPower(-(Input_Speed * Input_Angle * IMU_Slow_Turn));
//		   GobmotorBR1.setPower(-(Input_Speed * Input_Angle * IMU_Slow_Turn));
//		   GobmotorFL2.setPower(Input_Speed * Input_Angle * IMU_Slow_Turn);
//		   GobmotorBL3.setPower(Input_Speed * Input_Angle * IMU_Slow_Turn);
//		 } else if (2 == IMU_Polarity) {
//		   GobmotorFR0.setPower(Input_Speed * Input_Angle * IMU_Slow_Turn);
//		   GobmotorBR1.setPower(Input_Speed * Input_Angle * IMU_Slow_Turn);
//		   GobmotorFL2.setPower(-(Input_Speed * Input_Angle * IMU_Slow_Turn));
//		   GobmotorBL3.setPower(-(Input_Speed * Input_Angle * IMU_Slow_Turn));
//		 }
//	   }
//	 }
//	 GobmotorFR0.setPower(0);
//	 GobmotorBR1.setPower(0);
//	 GobmotorFL2.setPower(0);
//	 GobmotorBL3.setPower(0);
//   }
// }
// */
