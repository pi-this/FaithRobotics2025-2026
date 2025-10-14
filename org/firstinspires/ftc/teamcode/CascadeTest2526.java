package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
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
		DcMotor BL = hardwareMap.dcMotor.get("BL 2");
		DcMotor FR = hardwareMap.dcMotor.get("FR 1");
		DcMotor FL = hardwareMap.dcMotor.get("FL 3");
		
		
		pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "Pinpoint");
		//odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
		
		pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
		
		//pinpoint.doInitialize();
		pinpoint.setOffsets(95.25d, 12.7d);
		pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
		
		pinpoint.resetPosAndIMU();
		waitForStart();
		
		double speed = 500;
		double Am = 1000; // max acceleration, mm/s/s
		double switchDistance = speed*speed/Am;
		boolean isSwitched = false;
		
		
		
		double vkp = 0.002d;// 0.002
		double vki = 0.00002d;
	
		double vISum = 0;
		double vError = 0;
		double setpoint = 0d;
		double vfOut = 0;
		double targetPosPoint = 4_000d;
		double positionError = 0;
		
		ElapsedTime runTime = new ElapsedTime();
		runTime.reset();
		double oldTime = 0;
		
		
		while(opModeIsActive()){
			positionError = -pinpoint.getPosX() + targetPosPoint;
			
			if(isSwitched){
				setpoint = setpoint - (Am*(runTime.time()-oldTime));
				if(setpoint <= 0){
					setpoint = 0;
				}
			} else{
				setpoint = speed;
				if(positionError < switchDistance){
					isSwitched = true;// this is bad. only works if going forwards
				}
			}
			vError = -setpoint + pinpoint.getVelX();
			vISum += vError;
			vfOut = (vkp * vError) + (vki * vISum);
			
			
			BR.setPower(-vfOut);
			BL.setPower(vfOut);
			FR.setPower(-vfOut);
			FL.setPower(vfOut);
			oldTime = runTime.time();
			
			pinpoint.update();
			telemetry.addData("x", pinpoint.getPosX());
			telemetry.addData("ve", vError);
			telemetry.addData("out", vfOut);
			telemetry.addData("switched", isSwitched);
			telemetry.addData("setpoint", setpoint);
			telemetry.update();
			
		}
		
		//BR.setPower(1);
	}
	
	
	// todo: write your code here
}



