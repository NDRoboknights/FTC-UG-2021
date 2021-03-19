package org.firstinspires.ftc.teamcode.Auton.BlueThreeShot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.MecanumBot;
import org.firstinspires.ftc.teamcode.Robot.MecanumBotLocalizer;

public class BlueThreeShot extends LinearOpMode
{
    MecanumBot bot = new MecanumBot(hardwareMap);
    @Override
    public void runOpMode() throws InterruptedException
    {
        bot.init(hardwareMap);
        Trajectory trajectory = bot.rRDrive.trajectoryBuilder(new Pose2d(-64, 48, 0))
                .strafeTo(new Vector2d(-64, 0))
                .splineTo(new Vector2d(-60, 0), 0)
                .splineTo(new Vector2d(-18, 12), 0)
                .addDisplacementMarker(() ->
                    {
                        bot.s1.setVelocity(((MecanumBot.SHOOTER_VELOCITY * 28) / 60));
                        bot.s2.setVelocity(((MecanumBot.SHOOTER_VELOCITY * 28) / 60));

                        try {
                            wait(1500);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }

                        bot.hopper.setVelocity((MecanumBot.INTAKE_VELOCITY * MecanumBot.UP4_TICKS_PER_REVOLUTION) / 60);

                        try {
                            wait(50);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                        bot.hopper.setVelocity(0);
                    })
                .strafeTo(new Vector2d(-18, 8))
                .addDisplacementMarker(() ->
                    {
                        bot.hopper.setVelocity((MecanumBot.INTAKE_VELOCITY * MecanumBot.UP4_TICKS_PER_REVOLUTION) / 60);

                        try {
                            wait(50);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                        bot.hopper.setVelocity(0);
                    })
                .strafeTo(new Vector2d(-18, 4))
                .addDisplacementMarker(() ->
                    {
                        bot.hopper.setVelocity((MecanumBot.INTAKE_VELOCITY * MecanumBot.UP4_TICKS_PER_REVOLUTION) / 60);

                        try {
                            wait(50);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                        bot.hopper.setVelocity(0);
                    })
                .build();
        //bot.rRDrive.setLocalizer(new MecanumBotLocalizer());
        waitForStart();
        bot.rRDrive.followTrajectory(trajectory);
    }
}
