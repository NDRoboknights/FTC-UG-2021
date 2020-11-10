package org.firstinspires.ftc.teamcode.Robot;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class Bot
{
    public abstract void init(HardwareMap hMap);
    public abstract void setZPB(Motor.ZeroPowerBehavior zpb);

    public static double checkStick(double stickY){

        return 0;
    }
}
