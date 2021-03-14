package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot.MecanumBot;

@TeleOp(name = "FullTeleOp")
public class MecanumTeleControl extends OpMode {
	private static final double STICK_THRESH = 0.09375;


	MecanumBot bot;
	ShooterTest shooter;
	IntakeTest intake;

	double angle = 0;
	double power = 0;
	double turn = 0;

	@Override
	public void init()
	{
		bot = new MecanumBot(hardwareMap);
		bot.init(hardwareMap);

		shooter = new ShooterTest(bot, hardwareMap, gamepad1);
		intake = new IntakeTest(bot, hardwareMap, gamepad1);

		intake.init();
		shooter.init();
	}

	@Override
	public void loop()
	{
		angle = bot.getAngle(gamepad1.left_stick_y, gamepad1.left_stick_x) + Math.toRadians(45);
		turn = gamepad1.right_stick_x;
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

		bot.lFMotor.setPower(motorPowerFrontLeft);
		bot.rFMotor.setPower(motorPowerFrontRight);
		bot.lBMotor.setPower(motorPowerBackLeft);
		bot.rBMotor.setPower(motorPowerBackRight);

		shooter.loop();
		intake.loop();

		telemetry.addData("Strafe heading: ", Math.toDegrees(angle) - 45);
		telemetry.addData("Angle X: ", bot.imu.getXAxisValue());
		telemetry.addData("Angle Y: ", bot.imu.getYAxisValue());
		telemetry.addData("Angle Z: ", bot.imu.getZAxisValue());
		telemetry.addData("Left Trigger", gamepad1.left_trigger);
		telemetry.addData("Right Trigger", gamepad1.right_trigger);

		telemetry.update();
	}
}