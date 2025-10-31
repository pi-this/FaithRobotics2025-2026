// package org.firstinspires.ftc.teamcode.old;
// 
// import com.qualcomm.hardware.bosch.BNO055IMU;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
// import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
// 
// @TeleOp(name = "_1stProgrammingJosiah (Blocks to Java)")
// public class _1stProgrammingJosiah extends LinearOpMode {
// 
//   private BNO055IMU imu;
//   private DcMotor revFR0;
//   private DcMotor revBR1;
//   private DcMotor revFL2;
//   private DcMotor revBL3;
// 
//   /**
//    * This function is executed when this Op Mode is selected from the Driver Station.
//    */
//   @Override
//   public void runOpMode() {
//     float X_Angle;
//     float Setpoint;
//     BNO055IMU.Parameters IMU_Config_;
//     double Desired_Speed;
// 
//     imu = hardwareMap.get(BNO055IMU.class, "imu");
//     revFR0 = hardwareMap.get(DcMotor.class, "rev FR 0");
//     revBR1 = hardwareMap.get(DcMotor.class, "rev BR 1");
//     revFL2 = hardwareMap.get(DcMotor.class, "rev FL 2");
//     revBL3 = hardwareMap.get(DcMotor.class, "rev BL 3");
// 
//     waitForStart();
//     imu.initialize(new BNO055IMU.Parameters());
//     revFR0.setPower(0);
//     revBR1.setPower(0);
//     revFL2.setPower(0);
//     revBL3.setPower(0);
//     revFR0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//     revBR1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//     revFL2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//     revBL3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//     revFR0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//     revBR1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//     revFL2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//     revBL3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//     revFR0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//     revBR1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//     revFL2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//     revBL3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//     Desired_Speed = 0.5;
//     IMU_Config_ = new BNO055IMU.Parameters();
//     IMU_Config_.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//     IMU_Config_.mode = BNO055IMU.SensorMode.GYRONLY;
//     IMU_Config_.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//     telemetry.addData("IMU Status", imu.getSystemStatus().toString());
//     X_Angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
//     Setpoint = X_Angle;
//     telemetry.update();
//     if (isStarted()) {
//       while (true) {
//         telemetry.addData("Ticks", revFR0.getCurrentPosition());
//         X_Angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
//         revFR0.setPower(Desired_Speed + (X_Angle - Setpoint) / 100);
//         revBR1.setPower(Desired_Speed + (X_Angle - Setpoint) / 100);
//         revFL2.setPower(-Desired_Speed + (X_Angle - Setpoint) / 100);
//         revBL3.setPower(-Desired_Speed + (X_Angle - Setpoint) / 100);
//         revFR0.setTargetPosition(100000);
//         revBR1.setTargetPosition(100000);
//         revFL2.setTargetPosition(-100000);
//         revBL3.setTargetPosition(-100000);
//         telemetry.addData("X Orientation", X_Angle);
//         telemetry.addData("Set Point", Setpoint);
//         if (revFR0.getPower() == 0 || revBR1.getPower() == 0 || revFL2.getPower() == 0 || revBL3.getPower() == 0) {
//           revFR0.setPower(0);
//           revBR1.setPower(0);
//           revFL2.setPower(0);
//           revBL3.setPower(0);
//         }
//         telemetry.update();
//       }
//     }
//   }
// }
// 
