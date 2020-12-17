package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PID.VelocityPIDFController;

@TeleOp(name = "DualMotorShooterTest")
public class ShooterTest extends OpMode {

    DcMotorEx s1;
    DcMotorEx s2;

    double motorVelocity = 5000.00;

    //double kP, kI, kD, kV, kA, kStatic = 0;

    //VelocityPIDFController controller = new VelocityPIDFController(new PIDCoefficients(kP, kI, kD), kV, kA, kStatic);

    public void init()
    {
        s1 = hardwareMap.get(DcMotorEx.class, "s1");
        s2 = hardwareMap.get(DcMotorEx.class, "s2");
        s1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        s2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //unlocks built in velocity pid to 100% power
        MotorConfigurationType motorConfigurationType = s1.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        s1.setMotorType(motorConfigurationType);
        s2.setMotorType(motorConfigurationType);
    }

    public void loop()
    {
        s1.setVelocity(((motorVelocity * 28) / 60));
        s2.setVelocity(((motorVelocity * 28) / 60));

        if(gamepad1.dpad_down){
            motorVelocity -= 5;
        }

        if(gamepad1.dpad_up){
            motorVelocity += 5;
        }

        telemetry.addData("Nominal Velocity (rpm)", motorVelocity);
        telemetry.addData("Shooter 1 Velocity (rpm): ", (s1.getVelocity() / 28) * 60);
        telemetry.addData("Shooter 2 Velocity (rpm): ", (s2.getVelocity() / 28) * 60);

        telemetry.update();

    }
}
