package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "IntakeTest")
public class IntakeTest extends OpMode {

    DcMotorEx intake;

    public void init() 
    {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void loop()
    {
        intake.setPower(1);
        telemetry.addData("Intake Velocity: ", (intake.getVelocity() / 28) * 60);
        telemetry.update();
    }
}
