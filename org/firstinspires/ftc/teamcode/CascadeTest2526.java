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
	
	void drive(double dist, double speed, double acceleration, double tolerance, double target_angle){
		double switchDistance = speed*speed/acceleration; // trust the process
		boolean isSwitched = false;
		
		double errorAngle = 0;
		double kpAngle = 0.3;
		
		
		
		
		
		double vkp = 0.002d;// 0.002
		double vki = 0.00002d;
	
		double vISum = 0;
		double vError = 0;
		double setpoint = 0d;
		double vfOut = 0;
		double positionError = 0;
		
		ElapsedTime runTime = new ElapsedTime();
		runTime.reset();
		double oldTime = 0;
		
		int count = 0;
		
		double outputAngle = 0;
		
		
		while(opModeIsActive()){
			positionError = -pinpoint.getPosX() + dist;
			errorAngle = pinpoint.getHeading() - target_angle;
			
			outputAngle = -errorAngle * kpAngle;
			
			if(isSwitched){
				//setpoint = setpoint - (Am*(runTime.time()-oldTime));
				setpoint = speed*positionError/switchDistance*(Math.abs(speed)/speed);// Speed*percent*sign
				if(Math.abs(positionError) < tolerance){// could replace with abs, but is technically more consistent due to less branching
					count++;
					if(count > 50){
						break;
					}
				} else{
					count = 0;
				}
				/*if(setpoint <= 0){
					setpoint = 0;
				}*/
			} else{
				//setpoint = speed;
				setpoint = (Math.abs(setpoint) + (acceleration*(runTime.time()-oldTime))) * Math.abs(speed)/speed;
				if(Math.abs(setpoint) > Math.abs(speed)){// setpoint += acceleration * sign * dT
					setpoint = speed;
				}
				if(Math.abs(positionError) < switchDistance){
					isSwitched = true;
				}
			}
			vError = -setpoint + pinpoint.getVelX();
			vISum += vError;
			vfOut = (vkp * vError) + (vki * vISum);
			
			
			// right ones are backwards
			BR.setPower(-vfOut + outputAngle);
			BL.setPower(vfOut + outputAngle);
			FR.setPower(-vfOut + outputAngle);
			FL.setPower(vfOut + outputAngle);
			oldTime = runTime.time();
			
			pinpoint.update();
			telemetry.addData("x", pinpoint.getPosX());
			telemetry.addData("ve", vError);
			telemetry.addData("ae", errorAngle);
			telemetry.addData("out", vfOut);
			telemetry.addData("switched", isSwitched);
			telemetry.addData("setpoint", setpoint);
			telemetry.addData("switching distance", switchDistance);
			telemetry.update();
			
		}
	}
	
	
	
//	pinpoint.doInitialize();
	@Override
	public void runOpMode() /*throws InterruptedException*/ {
		BR = hardwareMap.dcMotor.get("BR 0");
		BL = hardwareMap.dcMotor.get("BL 2");
		FR = hardwareMap.dcMotor.get("FR 1");
		FL = hardwareMap.dcMotor.get("FL 3");
		
		
		pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "Pinpoint");
		//odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
		
		pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
		
		//pinpoint.doInitialize();
		pinpoint.setOffsets(95.25d, 12.7d);
		pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
		
		pinpoint.resetPosAndIMU();
		waitForStart();
		// distance, speed, acceleration, tolerance super not linear
		drive(-20000d, -200d, 2000d, 26d, 0d);
		//drive(-500, -500, 2000, 25);
		
		sleep(10);
		
		//drive(1, 500, 2000, 25);
		drive(0d, 200d, 2000d, 26d, 0d);
		
		
		/*
		double speed = 1500;
		double Am = 2000; // max acceleration, mm/s/s
		double switchDistance = speed*speed/Am;
		boolean isSwitched = false;
		
		
		
		double vkp = 0.002d;// 0.002
		double vki = 0.00002d;
	
		double vISum = 0;
		double vError = 0;
		double setpoint = 0d;
		double vfOut = 0;
		double targetPosPoint = 8_000d;
		double positionError = 0;
		
		ElapsedTime runTime = new ElapsedTime();
		runTime.reset();
		double oldTime = 0;
		
		
		while(opModeIsActive()){
			positionError = -pinpoint.getPosX() + targetPosPoint;
			
			if(isSwitched){
				//setpoint = setpoint - (Am*(runTime.time()-oldTime));
				setpoint = speed*(positionError/(switchDistance));
				
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
			telemetry.addData("denominator", (switchDistance));
			telemetry.update();
			
		}
		*/
		//BR.setPower(1);
	}
	
	
	// todo: write your code here
}



