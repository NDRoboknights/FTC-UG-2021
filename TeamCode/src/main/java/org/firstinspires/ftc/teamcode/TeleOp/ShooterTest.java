package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "ShooterTest")
public class ShooterTest extends OpMode {

    DcMotor s1;
    DcMotor s2;

    public void init()
    {
        s1 = hardwareMap.dcMotor.get("s1");
        s2 = hardwareMap.dcMotor.get("s2");
        s1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        s1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void loop()
    {
        s1.setPower(1);
        s2.setPower(1);
    }
}
