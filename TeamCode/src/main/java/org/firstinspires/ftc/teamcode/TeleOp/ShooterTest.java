package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "ShooterTest")
public class ShooterTest extends OpMode {

    MotorEx s1;
    MotorEx s2;

    public void init()
    {
        s1 = new MotorEx(hardwareMap, "s1", Motor.GoBILDA.BARE);
        s2 = new MotorEx(hardwareMap, "s2", Motor.GoBILDA.BARE);
    }

    public void loop()
    {
        s1.set(1);
        s1.set(1);
    }
}
