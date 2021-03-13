package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot.MecanumBot;


@TeleOp(name = "IntakeTest")
public class IntakeTest extends OpMode {

    HardwareMap hardware;
    MecanumBot bot;
    Gamepad gamepadF310;
    boolean aChanged = false;

    public IntakeTest(MecanumBot mecanumBot, HardwareMap hMap, Gamepad gamepad)
    {
        this.bot = mecanumBot;
        this.hardware = hMap;
        this.gamepadF310 = gamepad;
    }


    public void init()
    {
        bot = new MecanumBot(hardware);
        bot.init(hardware);
    }

    public void loop()
    {
        if(gamepadF310.a && !aChanged)
        {
            bot.intake.setVelocity(((MecanumBot.SHOOTER_VELOCITY * 28) / 60));
            aChanged = true;
        }else if(!gamepadF310.a)
        {
            aChanged = false;
        }
    }
}
