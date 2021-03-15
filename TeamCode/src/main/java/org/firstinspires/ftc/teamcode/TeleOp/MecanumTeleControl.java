package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot.MecanumBot;

@TeleOp(name = "FullTeleOp")
public class MecanumTeleControl extends OpMode {

	MecanumBot bot;
	ShooterTest shooter;
	IntakeTest intake;
	DrivetrainTest dt;

	@Override
	public void init()
	{
		bot = new MecanumBot(hardwareMap);
		bot.init(hardwareMap);

		shooter = new ShooterTest(bot, hardwareMap, gamepad1);
		intake = new IntakeTest(bot, hardwareMap, gamepad1);
		dt = new DrivetrainTest(bot, hardwareMap, gamepad1);

		dt.init();
		intake.init();
		shooter.init();
	}

	@Override
	public void loop()
	{
		dt.loop();
		shooter.loop();
		intake.loop();
	}
}