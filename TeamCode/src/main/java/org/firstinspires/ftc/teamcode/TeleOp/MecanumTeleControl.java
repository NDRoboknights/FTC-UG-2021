package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.MecanumBot;

import java.util.Scanner;

@TeleOp(name = "MecanumDrive")
public class MecanumTeleControl extends OpMode {
	private static final double STICK_THRESH = 0.09375;
	MecanumBot bot;

	double angle = 0;
	double power = 0;
	double turn = 0;

	@Override
	public void init()
	{
		bot = new MecanumBot(hardwareMap);
		bot.init(hardwareMap);
	}

	@Override
	public void loop()
	{
		angle = bot.getAngle(gamepad1.left_stick_y, gamepad1.left_stick_x);
		turn = gamepad1.right_stick_y;
		power = Math.sqrt(Math.pow(gamepad1.left_stick_y, 2) + Math.pow(gamepad1.left_stick_x, 2));

		double motorPowerFrontLeft = Math.cos(angle);
		double motorPowerFrontRight = Math.sin(angle);
		double motorPowerBackLeft = Math.sin(angle);
		double motorPowerBackRight = Math.cos(angle);

		// Normalize values
		double maxValue = Math.max(Math.abs(motorPowerFrontLeft),
				Math.max(Math.abs(motorPowerFrontRight),
						Math.max(Math.abs(motorPowerBackLeft), Math.abs(motorPowerBackRight))));

		motorPowerFrontLeft /= maxValue;
		motorPowerFrontRight /= maxValue;
		motorPowerBackLeft /= maxValue;
		motorPowerBackRight /= maxValue;

		motorPowerFrontLeft = motorPowerFrontLeft * power + turn;
		motorPowerFrontRight = motorPowerFrontRight * power - turn;
		motorPowerBackLeft = motorPowerBackLeft * power + turn;
		motorPowerBackRight = motorPowerBackRight * power - turn;

		bot.lFMotor.set(motorPowerFrontLeft);
		bot.rFMotor.set(motorPowerFrontRight);
		bot.lBMotor.set(motorPowerBackLeft);
		bot.rBMotor.set(motorPowerBackRight);

	}
}