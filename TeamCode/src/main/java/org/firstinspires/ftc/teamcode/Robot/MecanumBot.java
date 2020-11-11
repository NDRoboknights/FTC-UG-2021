package org.firstinspires.ftc.teamcode.Robot;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumBot extends Bot
{

	private MotorEx lFMotor;
	private MotorEx lBMotor;
	private MotorEx rFMotor;
	private MotorEx rBMotor;

	public MecanumDrive mecanumDrive;

	public MecanumBot(HardwareMap hardwareMap)
	{
		init(hardwareMap);
		setZPB(Motor.ZeroPowerBehavior.BRAKE);
	}

	public MecanumBot(HardwareMap hardwareMap, Motor.ZeroPowerBehavior zeroPowerBehavior)
	{
		init(hardwareMap);
		setZPB(zeroPowerBehavior);
	}

	public void init(HardwareMap hardwareMap)
	{
		lFMotor = new MotorEx(hardwareMap, "leftFront", Motor.GoBILDA.RPM_312);
		lBMotor = new MotorEx(hardwareMap, "leftBack", Motor.GoBILDA.RPM_312);
		rFMotor = new MotorEx(hardwareMap, "rightFront", Motor.GoBILDA.RPM_312);
		rBMotor = new MotorEx(hardwareMap, "rightBack", Motor.GoBILDA.RPM_312);
		mecanumDrive = new MecanumDrive(true, lFMotor, rFMotor, lBMotor, rBMotor);
	}

	public void setZPB(Motor.ZeroPowerBehavior zpb)
	{
		lFMotor.setZeroPowerBehavior(zpb);
		rFMotor.setZeroPowerBehavior(zpb);
		lBMotor.setZeroPowerBehavior(zpb);
		rBMotor.setZeroPowerBehavior(zpb);
	}

}
