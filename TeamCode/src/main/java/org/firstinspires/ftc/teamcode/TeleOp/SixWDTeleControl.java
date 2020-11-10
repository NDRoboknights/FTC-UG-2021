package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.SixWDBot;

@TeleOp(name = "6WDTest")
public class SixWDTeleControl extends OpMode {

    SixWDBot bot = null;

    @Override
    public void init() {
        bot = new SixWDBot(hardwareMap);
    }

    @Override
    public void loop() {

    }
}
