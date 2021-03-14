package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot.MecanumBot;

@TeleOp(name = "MecanumDrive")
public class DrivetrainTest extends OpMode {
    private static final double STICK_THRESH = 0.09375;


    private MecanumBot bot;
    private Gamepad gamer;
    private  HardwareMap map;

    double angle = 0;
    double power = 0;
    double turn = 0;
    
    public DrivetrainTest(MecanumBot  mecanumBot, HardwareMap hMap, Gamepad gpad){
        this.bot = mecanumBot;
        this.map = hMap;
        this.gamer = gpad;
    }

    @Override
    public void init()
    {
        bot = new MecanumBot(map);
        bot.init(map);
    }

    @Override
    public void loop()
    {
        //noinspection SuspiciousNameCombination
        angle = bot.getAngle(gamer.left_stick_y, gamer.left_stick_x) + Math.toRadians(45);
        turn = gamer.right_stick_x;
        power = Math.sqrt(Math.pow(gamer.left_stick_y, 2) + Math.pow(gamer.left_stick_x, 2));

        double motorPowerFrontLeft = Math.cos(angle);
        double motorPowerFrontRight = Math.sin(angle);
        double motorPowerBackLeft = Math.sin(angle);
        double motorPowerBackRight = Math.cos(angle);

        // Normalize values
        double maxValue = Math.max(Math.abs(motorPowerFrontLeft),
                Math.max(Math.abs(motorPowerFrontRight),
                        Math.max(Math.abs(motorPowerBackLeft), Math.abs(motorPowerBackRight))));

        motorPowerFrontLeft /= maxValue;
        motorPowerFrontRight /= maxValue;
        motorPowerBackLeft /= maxValue;
        motorPowerBackRight /= maxValue;

        motorPowerFrontLeft = motorPowerFrontLeft * power + turn;
        motorPowerFrontRight = motorPowerFrontRight * power - turn;
        motorPowerBackLeft = motorPowerBackLeft * power + turn;
        motorPowerBackRight = motorPowerBackRight * power - turn;

        bot.lFMotor.setPower(motorPowerFrontLeft);
        bot.rFMotor.setPower(motorPowerFrontRight);
        bot.lBMotor.setPower(motorPowerBackLeft);
        bot.rBMotor.setPower(motorPowerBackRight);

        telemetry.addData("Strafe heading: ", Math.toDegrees(angle) - 45);
        telemetry.addData("Angle X: ", bot.imu.getXAxisValue());
        telemetry.addData("Angle Y: ", bot.imu.getYAxisValue());
        telemetry.addData("Angle Z: ", bot.imu.getZAxisValue());

        telemetry.update();
    }
}