package org.firstinspires.ftc.teamcode.Robot;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumBot extends Bot
{

	public MotorEx lFMotor;
	public MotorEx lBMotor;
	public MotorEx rFMotor;
	public MotorEx rBMotor;

	public MecanumBot(HardwareMap hardwareMap)
	{
		init(hardwareMap);
		setZPB(Motor.ZeroPowerBehavior.BRAKE);
	}

	public MecanumBot(HardwareMap hardwareMap, Motor.ZeroPowerBehavior zeroPowerBehavior)
	{
		//init(hardwareMap);
		//setZPB(zeroPowerBehavior);
	}

	public void init(HardwareMap hardwareMap)
	{
		lFMotor = new MotorEx(hardwareMap, "leftFront", Motor.GoBILDA.RPM_312);
		lBMotor = new MotorEx(hardwareMap, "leftBack", Motor.GoBILDA.RPM_312);
		rFMotor = new MotorEx(hardwareMap, "rightFront", Motor.GoBILDA.RPM_312);
		rBMotor = new MotorEx(hardwareMap, "rightBack", Motor.GoBILDA.RPM_312);
	}

	public void setZPB(Motor.ZeroPowerBehavior zpb)
	{
		lFMotor.setZeroPowerBehavior(zpb);
		rFMotor.setZeroPowerBehavior(zpb);
		lBMotor.setZeroPowerBehavior(zpb);
		rBMotor.setZeroPowerBehavior(zpb);
	}

	public double getAngle(double xComponent, double yComponent){
		return Math.toDegrees(Math.atan(yComponent / xComponent));
	}
}
