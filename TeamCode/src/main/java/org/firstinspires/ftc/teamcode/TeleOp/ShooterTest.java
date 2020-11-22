package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "ShooterTest")
public class ShooterTest extends OpMode {

    DcMotorEx s1;
    DcMotorEx s2;

    public void init()
    {
        s1 = hardwareMap.get(DcMotorEx.class, "s1");
        s2 = hardwareMap.get(DcMotorEx.class, "s2");
        s1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        s1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void loop()
    {
        s1.setPower(1);
        s2.setPower(1);
        telemetry.addData("Shooter 1 Velocity: ", s1.getVelocity(AngleUnit.DEGREES) * (1.0/6.0));
        telemetry.addData("Shooter 2 Velocity: ", s1.getVelocity(AngleUnit.DEGREES) * (1.0/6.0));

        telemetry.update();
    }
}
