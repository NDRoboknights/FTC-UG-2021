package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot.MecanumBot;


@TeleOp(name = "DualMotorShooterTest")
public class ShooterTest extends OpMode {

    HardwareMap hardware;
    MecanumBot bot;
    Gamepad gamepadF310;
    boolean xChanged, bChanged = false;

    public ShooterTest(MecanumBot mecanumBot, HardwareMap hMap, Gamepad gamepad)
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
        if(gamepadF310.x && !xChanged)
        {
            bot.s1.setVelocity(((MecanumBot.SHOOTER_VELOCITY *
                    MecanumBot.BARE_TICKS_PER_REVOLUTION) / 60));
            bot.s2.setVelocity(((MecanumBot.SHOOTER_VELOCITY *
                    MecanumBot.BARE_TICKS_PER_REVOLUTION) / 60));
            xChanged = true;
        }else if(!gamepadF310.x)
            {
            xChanged = false;
        }

        if(gamepadF310.b && !bChanged)
        {
            bot.hopper.setPower(1);
            bChanged = true;
        }else if(!gamepadF310.b)
            {
            bChanged = false;
        }
        
//        telemetry.addData("Nominal Velocity (rpm)", MecanumBot.SHOOTER_VELOCITY);
//        telemetry.addData("Shooter 1 Velocity (rpm): ", (bot.s1.getVelocity() / 28) * 60);
//        telemetry.addData("Shooter 2 Velocity (rpm): ", (bot.s2.getVelocity() / 28) * 60);
//
//        telemetry.update();
    }
}
