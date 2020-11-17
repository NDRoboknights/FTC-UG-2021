package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumBot extends Bot
{

	public DcMotorEx lFMotor;
	public DcMotorEx lBMotor;
	public DcMotorEx rFMotor;
	public DcMotorEx rBMotor;

	public MecanumBot(HardwareMap hardwareMap)
	{
		init(hardwareMap);
		setZPB(DcMotorEx.ZeroPowerBehavior.BRAKE);
	}

	public MecanumBot(HardwareMap hardwareMap, DcMotorEx.ZeroPowerBehavior zeroPowerBehavior)
	{
		init(hardwareMap);
		setZPB(zeroPowerBehavior);
	}

	public void init(HardwareMap hardwareMap)
	{
		lFMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
		lFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		lFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
		lBMotor = hardwareMap.get(DcMotorEx.class, "leftBack");
		lBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		lBMotor.setDirection(DcMotorSimple.Direction.REVERSE);
		rFMotor = hardwareMap.get(DcMotorEx.class, "rightFront");
		rFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		rBMotor = hardwareMap.get(DcMotorEx.class, "rightBack");
		rBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	}

	public void setZPB(DcMotor.ZeroPowerBehavior zpb)
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
