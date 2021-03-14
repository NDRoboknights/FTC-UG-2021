package org.firstinspires.ftc.teamcode.Robot;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.roboknights4348.lib.wpimath.src.main.java.edu.wpi.first.wpilibj.kinematics.MecanumDriveKinematics;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.REVHubIMU;

public class MecanumBot extends Bot
{

	//Constants
	public static final double SHOOTER_VELOCITY = 4800;
	public static final double INTAKE_VELOCITY = 1150;
	public static final double DRIVE_VELOCITY = 312;

	public static final int BARE_TICKS_PER_REVOLUTION = 28;
	public static final double YJ1150_TICKS_PER_REVOLUTION = 145.6;
	public static final double YJ312_TICKS_PER_REVOLUTION = 537.6;

	public static final double encoderMeasurementCovariance = 0.75;

	public static final Transform2d cameraRelativePosition = new Transform2d(
	        new Translation2d(0.14475, 0.20050), new Rotation2d()
    );

	public DcMotorEx lFMotor;
	public DcMotorEx lBMotor;
	public DcMotorEx rFMotor;
	public DcMotorEx rBMotor;

	public DcMotorEx intake;

	public REVHubIMU imu;
	private BNO055IMU bno055IMU;
	public Rev2mDistanceSensor xDist;
	public Rev2mDistanceSensor yDist;

	public DcMotorEx s1;
	public DcMotorEx s2;
	public DcMotorEx hopper;

	public Servo hopperServo;

	public MecanumDriveKinematics mecanumDriveKinematics = new MecanumDriveKinematics(
			new Translation2d(-0.217, 0.168),
			new Translation2d(0.217, 0.168),
			new Translation2d(-0.217, -0.168),
			new Translation2d(0.217, -0.168));

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
		//drive motors
		lFMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
		lFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		lBMotor = hardwareMap.get(DcMotorEx.class, "leftBack");
		lBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		rFMotor = hardwareMap.get(DcMotorEx.class, "rightFront");
		rFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
		rBMotor = hardwareMap.get(DcMotorEx.class, "rightBack");
		rBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		rBMotor.setDirection(DcMotorSimple.Direction.REVERSE);

		//intake motor
		intake = hardwareMap.get(DcMotorEx.class, "intake");
		intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		//servo for hopper
		hopperServo = hardwareMap.servo.get("hopperServo");

		//shooter motors
		hopper = hardwareMap.get(DcMotorEx.class, "hopper");
		s1 = hardwareMap.get(DcMotorEx.class, "s1");
		s2 = hardwareMap.get(DcMotorEx.class, "s2");
		s1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		s2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		hopper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		//unlocks built in velocity pid to 100% power
		MotorConfigurationType motorConfigurationType = s1.getMotorType().clone();
		motorConfigurationType.setAchieveableMaxRPMFraction(1);
		s1.setMotorType(motorConfigurationType);
		s2.setMotorType(motorConfigurationType);
		hopper.setMotorType(motorConfigurationType);
		intake.setMotorType(motorConfigurationType);
		lFMotor.setMotorType(motorConfigurationType);
		lBMotor.setMotorType(motorConfigurationType);
		rFMotor.setMotorType(motorConfigurationType);
		rBMotor.setMotorType(motorConfigurationType);

		imu = new REVHubIMU(hardwareMap, "imu", bno055IMU);
		xDist = hardwareMap.get(Rev2mDistanceSensor.class, "xDist");
		yDist = hardwareMap.get(Rev2mDistanceSensor.class, "yDist");
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

	public void driveForTime(double power, long time)
	{
		lFMotor.setPower(power);
		lBMotor.setPower(power);
		rFMotor.setPower(power);
		rBMotor.setPower(power);

		try {
			Thread.sleep(time);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}

		lFMotor.setPower(0);
		lBMotor.setPower(0);
		rFMotor.setPower(0);
		rBMotor.setPower(0);
	}

	public MecanumDriveWheelSpeeds getWheelSpeeds()
	{
		return new MecanumDriveWheelSpeeds(lFMotor.getVelocity(AngleUnit.RADIANS)*0.096,
				rFMotor.getVelocity(AngleUnit.RADIANS)*0.096,
				lBMotor.getVelocity(AngleUnit.RADIANS)*0.096,
				rBMotor.getVelocity(AngleUnit.RADIANS)*0.096);
	}

}
