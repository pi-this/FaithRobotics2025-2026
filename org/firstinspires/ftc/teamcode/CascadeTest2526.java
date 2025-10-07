package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Gyroscope;
//import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;


@Autonomous

public class CascadeTest2526 extends LinearOpMode{
	private DcMotor BL;
	private DcMotor BR;
	private Blinker control_Hub;
	private DcMotor FL;
	private DcMotor FR;
	private HardwareDevice webcam_1;
	private Gyroscope imu;
	GoBildaPinpointDriver pinpoint;
	
	
	
	
//	pinpoint.doInitialize();
	@Override
	public void runOpMode() /*throws InterruptedException*/ {
		DcMotor BR = hardwareMap.dcMotor.get("BR 0");
		
		
		pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "Pinpoint");
		//odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
		
		pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
		
		//pinpoint.doInitialize();
		pinpoint.setOffsets(3.75d, 0.5d);
		pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
		
		pinpoint.resetPosAndIMU();
		waitForStart();
		
		while(true){
			pinpoint.update();
			telemetry.addData("x", pinpoint.getPosX());
			telemetry.update();
		}
		
		//BR.setPower(1);
	}
	
	
	// todo: write your code here
}



