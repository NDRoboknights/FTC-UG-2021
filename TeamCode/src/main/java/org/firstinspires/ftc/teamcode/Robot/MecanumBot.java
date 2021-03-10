package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.util.REVHubIMU;

public class MecanumBot extends Bot
{

	public DcMotorEx lFMotor;
	public DcMotorEx lBMotor;
	public DcMotorEx rFMotor;
	public DcMotorEx rBMotor;

	public DcMotorEx intake;

	public REVHubIMU imu;
	private BNO055IMU bno055IMU;

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
		lBMotor = hardwareMap.get(DcMotorEx.class, "leftBack");
		lBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		rFMotor = hardwareMap.get(DcMotorEx.class, "rightFront");
		rFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		rFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
		rBMotor = hardwareMap.get(DcMotorEx.class, "rightBack");
		rBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		rBMotor.setDirection(DcMotorSimple.Direction.REVERSE);

		intake = hardwareMap.get(DcMotorEx.class, "intake");
		intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		MotorConfigurationType motorConfigurationType = intake.getMotorType().clone();
		motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
		intake.setMotorType(motorConfigurationType);

		imu = new REVHubIMU(hardwareMap, "imu", bno055IMU);
	}

	public void setZPB(DcMotor.ZeroPowerBehavior zpb)
	{
		lFMotor.setZeroPowerBehavior(zpb);
		rFMotor.setZeroPowerBehavior(zpb);
		lBMotor.setZeroPowerBehavior(zpb);
		rBMotor.setZeroPowerBehavior(zpb);
	}

	public double getAngle(double xComponent, double yComponent){
		return Math.atan2(yComponent , xComponent);
	}
}
