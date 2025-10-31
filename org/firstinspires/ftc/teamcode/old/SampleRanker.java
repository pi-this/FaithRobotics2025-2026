// package org.firstinspires.ftc.teamcode.old;
// 
// 
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.util.ElapsedTime;
// 
// import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
// import java.lang.reflect.Array;
// import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
// import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
// import org.opencv.core.Point;
// import org.opencv.core.RotatedRect;
// import java.util.List;
// import java.util.ArrayList;
// 
// 
// 
// 
// 
// public class SampleRanker extends BlocksOpModeCompanion {
// 
// @ExportToBlocks(
// 		tooltip = "Returns the distance between two points",
// 		parameterLabels = {"ax","ay","bx","by"}
// )
// public static double distance(double ax, double ay, double bx, double by){
// 	return Math.sqrt(Math.pow(bx-ax,2) + Math.pow(by-ay,2));
// }
// 
// @ExportToBlocks()
// public static double get(List<Double> arr, int i){
// 	return arr.get(i);
// }
// 
// @ExportToBlocks(
// 		tooltip = "Returns the best sample for collection",
// 		parameterLabels = {"isRed", "RedSamples", "BlueSamples", "YellowSamples", "isOutputtingIN"}
// )//List<Double>
// public static Point Rank(boolean isRed, ColorBlobLocatorProcessor redP, ColorBlobLocatorProcessor blueP, ColorBlobLocatorProcessor yellowP, boolean isOutputtingIN){
// 	//if(reds)
// 	telemetry.update();
// 	// must do it this way. you cant pass Blobs through the function arguments
// 	List<ColorBlobLocatorProcessor.Blob> blues = new ArrayList<ColorBlobLocatorProcessor.Blob>();
// 	List<ColorBlobLocatorProcessor.Blob> reds = new ArrayList<ColorBlobLocatorProcessor.Blob>();
// 	List<ColorBlobLocatorProcessor.Blob> yellows = new ArrayList<ColorBlobLocatorProcessor.Blob>();
// 	
// 	reds = redP.getBlobs();
// 	blues = blueP.getBlobs();
// 	yellows = yellowP.getBlobs();
// 	
// 	
// 	Point answer = new Point();
// 	answer.x = -10000.0f;
// 	answer.y = -10000.0f;
// 	
// 	double minDist = 150.0f; // the minimum distance a sample must be from an obstacle to be a candidate
// 	
// 	// get the number of potential samples that are not yellow
// 	int len = yellows.size();
// 	if(isRed){
// 		len += reds.size();
// 	} else{
// 		len += blues.size();
// 	}
// 	
// 	double[][] targets = new double[len][4];// x,y,
// 	double[][] nonTargets = new double[isRed ? blues.size() : reds.size()][2];
// 	
// 	
// 	
// 	
// 	int k = 0;// yellows
// 	for(int i=0;i<yellows.size();++i) {
// 		targets[k][0] = yellows.get(i).getBoxFit().center.x;
// 		targets[k][1] = yellows.get(i).getBoxFit().center.y;
// 		k++;
// 	}
// 	
// 	
// 	
// 	if(isRed){
// 		for(int i=0;i<reds.size();++i) {
// 			targets[k][0] = reds.get(i).getBoxFit().center.x;
// 			targets[k][1] = reds.get(i).getBoxFit().center.y;
// 			k++;
// 		}
// 	} else{
// 		for(int i=0;i<blues.size();++i) {
// 			targets[k][0] = blues.get(i).getBoxFit().center.x;
// 			targets[k][1] = blues.get(i).getBoxFit().center.y;
// 			k++;
// 		}
// 	}
// 	
// 	
// 	
// 	k = 0;// dont modify after this point. next loop depends on it
// 	// get non targets positions
// 	for(ColorBlobLocatorProcessor.Blob b : isRed ? blues : reds) {
// 		nonTargets[k][0] = b.getBoxFit().center.x;
// 		nonTargets[k][1] = b.getBoxFit().center.y;
// 		k++;
// 	}
// 	
// 	for(int i=0; i<len; ++i){// find closest dist between each obstacle and target
// 		targets[i][3] = 100000000;// infinity
// 		for(int j=0; j<k; ++j){
// 			double dist = distance(nonTargets[j][0],nonTargets[j][1],targets[i][0],targets[i][1]);
// 			
// 			
// 			if(targets[i][3] > dist){
// 				targets[i][3] = dist;
// 			}
// 		}
// 	}
// 	
// 	for(int i=0; i<len; ++i){// find closest dist for each potential target
// 		targets[i][2] = 100000000;// infinity
// 		for(int j=0; j<len; ++j){
// 			double dist = distance(targets[j][0],targets[j][1],targets[i][0],targets[i][1]);
// 			
// 			
// 			if(targets[i][2] > dist && dist != 0){
// 				targets[i][2] = dist;// assign closest dist to each potential target
// 			}
// 		}
// 	}
// 	
// 	
// 	// the number of candidates
// 	List<Double> candidates = new ArrayList<>();
// 	int l = 0;
// 	
// 	List<Double> dists = new ArrayList<>();
// 	
// 	for(int i=0; i<len; ++i){// assumes that collector only collects one sample
// 		if(targets[i][3] > minDist){
// 			candidates.add(l,targets[i][0]);
// 			candidates.add(l+1,targets[i][1]);
// 			dists.add(distance(targets[i][0],targets[i][1],420,240));// should be trying to go to a point that is slightly closer to the robot than the center of the screen
// 			l += 2;
// 		}
// 	}//cam is 640x480
// 	
// 	for(int i=0;i<dists.size(); ++i){
// 		double min = 100000000; // infinity
// 		int index = 0;
// 		for(int j=i;j<dists.size(); ++j){
// 			if(dists.get(j) < min){
// 				index = j;
// 				min = dists.get(j);
// 			}
// 			double temp;
// 			double tempx;
// 			double tempy;
// 			
// 			temp = dists.get(i);
// 			tempx = candidates.get(i*2);
// 			tempy = candidates.get((i*2)+1);
// 			
// 			dists.set(i,dists.get(index));
// 			candidates.set(i*2,candidates.get(index*2));
// 			candidates.set((i*2)+1,candidates.get((index*2)+1));
// 			
// 			dists.set(index, temp);
// 			candidates.set(index*2, tempx);
// 			candidates.set((index*2)+1, tempy);
// 			
// 		}
// 	}
// 	
// 	
// 	
// 	if(candidates.size() > 0){
// 		answer.x = candidates.get(0);
// 		answer.y = candidates.get(1);
// 	}
// 	
// 	
// 	/*List<Double> ans = new ArrayList<>();
// 	ans.add(answer.x); only necessary if using blocks. for some reason, blocks doesnt
// 	ans.add(answer.y); like passing anything of type Point and will crash if you try to access an element (printing the entire datastructure will look good, it only crashes on the access)*/
// 	
// 	//telemetry.addData("Dist", (-0.0335489086*answer.x) + 32.71806791/*lin regression between closest and furthest points that a sample is visible from*/);// seems to be less accurate than the width estimation. probably nonlinear
// 	//telemetry.addData("Width", ((-0.0151098901*answer.y) + 3.938873626) * ((-0.0010831915*answer.x) + 1.67753631));// left is lin reg between the leftmost and rightmost points that are as close as possible. right is the lin reg of the scale factor that occurs between the br point and the fr point. for example at br, it might be 6in to the right, but fr would be 9 and thus the scale factor is 1.5
// 	//telemetry.update();
// 	if(isOutputtingIN){
// 		answer.x = ((-0.0335489086*answer.x) + 32.71806791) - 18.0;// 18 is target dist between bot and the sample
// 		answer.y = ((-0.0151098901*answer.y) + 3.938873626) * ((-0.0010831915*answer.x) + 1.67753631);
// 	}
// 	
// 	
// 	return answer;
// }
// 
// @ExportToBlocks(
// 	parameterLabels = {"Speed","Angle","Tolerance","isRed","RedProcessor", "BlueProcessor", "YellowProcessor"}
// 	)
// public static void GOTOcamSample(double Speed, double Angle, double Tolerance, boolean isRed, ColorBlobLocatorProcessor redP, ColorBlobLocatorProcessor blueP, ColorBlobLocatorProcessor yellowP){
// 	DcMotor FL3;
//   DcMotor BL2;
//   DcMotor FR1;
//   DcMotor BR0;
//   
//   FL3 = hardwareMap.get(DcMotor.class, "FL 3");
//   BL2 = hardwareMap.get(DcMotor.class, "BL 2");
//   FR1 = hardwareMap.get(DcMotor.class, "FR 1");
//   BR0 = hardwareMap.get(DcMotor.class, "BR 0");
// 	
// 	double Kpfb = 0.001;
// 	
// 	double FBcorrection = 0;//forward backward correction
// 	double StrafeCorrection = 0;
// 	double AngleCorrection = 0;
// 	
// 	Point target = Rank(isRed, redP, blueP, yellowP, false);
// 	
// 	ElapsedTime elapsedTime = new ElapsedTime();
// 	
// 	// check if target position is null?
// 	Point p = new Point();
// 	p.x = 332.0d;
// 	p.y = 235.0d;
// 	while(elapsedTime.milliseconds() < 3000 && Tolerance < distance(target.x,target.y,332.0d,235.0d)){
// 		target = Track(target, isRed, redP, blueP, yellowP, false);
// 		
// 		if(target.x != -10000.0f){
// 			FBcorrection = (target.x - 332.0d) * 0.004;//332.0d
// 			AngleCorrection = 0;
// 			StrafeCorrection = (target.y - 235.0d) * -0.004;
// 		} else {
// 			FBcorrection = 0;
// 			AngleCorrection = 0;
// 			StrafeCorrection = 0;
// 		}
// 		
// 		
// 		telemetry.addData("ex", target.x - p.x);
// 		telemetry.addData("ey", target.y - p.y);
// 		telemetry.addData("FBOut", FBcorrection);
// 		telemetry.addData("SOut", StrafeCorrection);
// 		telemetry.update();
// 		
// 		FL3.setPower(Math.min(Math.max(FBcorrection + AngleCorrection + -StrafeCorrection, -Speed), Speed));
// 		BL2.setPower(Math.min(Math.max(FBcorrection + AngleCorrection + StrafeCorrection, -Speed), Speed));
// 		FR1.setPower(Math.min(Math.max(-FBcorrection + AngleCorrection + -StrafeCorrection, -Speed), Speed));
// 		BR0.setPower(Math.min(Math.max(-FBcorrection + AngleCorrection + StrafeCorrection, -Speed), Speed));
// 		
// 	}
// 	
// 	FL3.setPower(0.0);
// 	BL2.setPower(0.0);
// 	FR1.setPower(0.0);
// 	BR0.setPower(0.0);
// 	
// 	return;
// }
// 
// @ExportToBlocks(
// 	parameterLabels = {"Speed","Angle","Tolerance","xyRelative"}
// 	)
// public static void GOTODriveRelative(double Speed, double Angle, double Tolerance, Point xy){
// 	DcMotor FL3;
// 	DcMotor BL2;
// 	DcMotor FR1;
// 	DcMotor BR0;
// 	
// 	FL3 = hardwareMap.get(DcMotor.class, "FL 3");
// 	BL2 = hardwareMap.get(DcMotor.class, "BL 2");
// 	FR1 = hardwareMap.get(DcMotor.class, "FR 1");
// 	BR0 = hardwareMap.get(DcMotor.class, "BR 0");
// 	
// 	double FBcorrection = 0;//forward backward correction
// 	double StrafeCorrection = 0;
// 	double AngleCorrection = 0;
// 	
// 	Point target = new Point();
// 	target.x = xy.x;
// 	target.y = xy.y;
// 	
// 	PinpointBlocks.update();
// 	Point origin = new Point();
// 	origin.x = PinpointBlocks.xPosition(DistanceUnit.INCH);
// 	origin.y = PinpointBlocks.yPosition(DistanceUnit.INCH);
// 	
// 	
// 	ElapsedTime elapsedTime = new ElapsedTime();
// 	//elapsedTime = myElapsedTime.milliseconds();
// 	//elapsedTime.reset();
// 	
// 	while(elapsedTime.milliseconds() < 1000 && Tolerance < distance(target.x,target.y,(origin.x - PinpointBlocks.xPosition(DistanceUnit.INCH)),(origin.y - PinpointBlocks.yPosition(DistanceUnit.INCH)))){
// 		PinpointBlocks.update();
// 		PinpointBlocks.xPosition(DistanceUnit.INCH);
// 		
// 		FBcorrection = ((origin.x - PinpointBlocks.xPosition(DistanceUnit.INCH)) - target.x) * 0.144;// add relative x position
// 		AngleCorrection = 0;
// 		StrafeCorrection = ((origin.y - PinpointBlocks.yPosition(DistanceUnit.INCH)) - target.y) * 0.144;// add relative y position
// 		
// 		telemetry.addData("ex", (origin.x - PinpointBlocks.xPosition(DistanceUnit.INCH)) - target.x);
// 		telemetry.addData("ey", (origin.y - PinpointBlocks.yPosition(DistanceUnit.INCH)) - target.y);
// 		telemetry.addData("xr", origin.x - PinpointBlocks.xPosition(DistanceUnit.INCH));
// 		telemetry.addData("yr", origin.y - PinpointBlocks.xPosition(DistanceUnit.INCH));
// 		telemetry.addData("FBOut", FBcorrection);
// 		telemetry.addData("SOut", StrafeCorrection);
// 		telemetry.update();
// 		
// 		FL3.setPower(Math.min(Math.max(FBcorrection + AngleCorrection + -StrafeCorrection, -Speed), Speed));
// 		BL2.setPower(Math.min(Math.max(FBcorrection + AngleCorrection + StrafeCorrection, -Speed), Speed));
// 		FR1.setPower(Math.min(Math.max(-FBcorrection + AngleCorrection + -StrafeCorrection, -Speed), Speed));
// 		BR0.setPower(Math.min(Math.max(-FBcorrection + AngleCorrection + StrafeCorrection, -Speed), Speed));
// 		
// 	}
// 	
// 	FL3.setPower(0.0);
// 	BL2.setPower(0.0);
// 	FR1.setPower(0.0);
// 	BR0.setPower(0.0);
// 		
// 	
// 	return;
// }
// 
// @ExportToBlocks(
// 	
// )
// static public Point Track(Point previousPos, boolean isRed, ColorBlobLocatorProcessor redP, ColorBlobLocatorProcessor blueP, ColorBlobLocatorProcessor yellowP, boolean isOutputtingIN){
// 	Point answer = new Point();
// 	answer.x = -10000.0;
// 	answer.y = -10000.0;
// 	
// 	List<ColorBlobLocatorProcessor.Blob> blues = new ArrayList<ColorBlobLocatorProcessor.Blob>();
// 	List<ColorBlobLocatorProcessor.Blob> reds = new ArrayList<ColorBlobLocatorProcessor.Blob>();
// 	List<ColorBlobLocatorProcessor.Blob> yellows = new ArrayList<ColorBlobLocatorProcessor.Blob>();
// 	
// 	reds = redP.getBlobs();
// 	blues = blueP.getBlobs();
// 	yellows = yellowP.getBlobs();
// 	
// 	int len = yellows.size();
// 	if(isRed){
// 		len += reds.size();
// 	} else{
// 		len += blues.size();
// 	}
// 	
// 	double[][] targets = new double[len][2];// x,y,
// 	double[][] nonTargets = new double[isRed ? blues.size() : reds.size()][2];
// 	
// 	
// 	
// 	int k = 0;// yellows
// 	for(int i=0;i<yellows.size();++i) {
// 		targets[k][0] = yellows.get(i).getBoxFit().center.x;
// 		targets[k][1] = yellows.get(i).getBoxFit().center.y;
// 		k++;
// 	}
// 	
// 	
// 	if(isRed){
// 		for(int i=0;i<reds.size();++i) {
// 			targets[k][0] = reds.get(i).getBoxFit().center.x;
// 			targets[k][1] = reds.get(i).getBoxFit().center.y;
// 			k++;
// 		}
// 	} else{
// 		for(int i=0;i<blues.size();++i) {
// 			targets[k][0] = blues.get(i).getBoxFit().center.x;
// 			targets[k][1] = blues.get(i).getBoxFit().center.y;
// 			k++;
// 		}
// 	}
// 	
// 	
// 	
// 	k = 0;// dont modify after this point. next loop depends on it
// 	// get non targets positions
// 	
// 	for(ColorBlobLocatorProcessor.Blob b : isRed ? blues : reds) {
// 		nonTargets[k][0] = b.getBoxFit().center.x;
// 		nonTargets[k][1] = b.getBoxFit().center.y;
// 		k++;
// 	}
// 	
// 	double dist = 100000000.0;//infinity
// 	int index = 0;
// 	for(int i=0; i<len; ++i){
// 		if(distance(targets[i][0],targets[i][1],previousPos.x,previousPos.y) < dist){
// 			dist = distance(targets[i][0],targets[i][1],previousPos.x,previousPos.y);
// 			index = i;
// 		}
// 	}
// 	
// 	if(dist != 100000000.0){
// 		answer.x = targets[index][0];
// 		answer.y = targets[index][1];
// 	}
// 	
// 	
// 	return answer;
// }
// }
// 