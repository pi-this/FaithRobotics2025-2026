// package org.firstinspires.ftc.teamcode;
// 
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.Blinker;
// import com.qualcomm.robotcore.hardware.HardwareDevice;
// import com.qualcomm.robotcore.hardware.Gyroscope;
// 
// 
// public class PID {
// 	private DcMotor bL_2;
// 	private DcMotor bR_0;
// 	private Blinker control_Hub;
// 	private DcMotor fL_3;
// 	private DcMotor fR_1;
// 	private HardwareDevice webcam_1;
// 	private Gyroscope imu;
// 	
// 	DcMotor br;
// 	DcMotor bl;
// 	DcMotor fr;
// 	DcMotor fl;
// 
// 	// todo: write your code here
// 	
// 	
// 	@Override
// 	public void runOpMode();
// 	{
// 		br = hardwareMap.get(DcMotor.class, "BR 0");
// 		bl = hardwareMap.get(DcMotor.class, "BL 2");
// 		fr = hardwareMap.get(DcMotor.class, "FR 1");
// 		fl = hardwareMap.get(DcMotor.class, "FL 3");
// 		
// 		while(true){
// 			
// 		}
// 		
// 		br.setPower(stepPower);
// 		fr.setPower(stepPower);
// 		bl.setPower(-stepPower);
// 		fl.setPower(-stepPower);
// 	}
// }
// 	
// 	
// 	
// 	
// 	