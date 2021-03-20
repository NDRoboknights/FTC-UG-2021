package org.firstinspires.ftc.teamcode.Auton.Localizer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.MecanumBot;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

public class Ftc265Localizer implements Localizer
{
    private MecanumBot bot;
    private Telemetry telemetry;
    private T265Camera.CameraUpdate cameraUpdate;

    public Ftc265Localizer(HardwareMap hMap, Telemetry telem)
    {
        this.bot = new MecanumBot(hMap);
        this.telemetry = telem;
        bot.slamera.start();
    }
    @NotNull
    @Override
    public Pose2d getPoseEstimate()
    {
         cameraUpdate = bot.slamera.getLastReceivedCameraUpdate();
        return new Pose2d(cameraUpdate.pose.getX(), cameraUpdate.pose.getY(), cameraUpdate.pose.getHeading());
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d)
    {
        bot.slamera.setPose(new com.arcrobotics.ftclib.geometry.Pose2d(
                new Translation2d(pose2d.getX(), pose2d.getY()),
                new Rotation2d(pose2d.getHeading())
        ));
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity()
    {
        return null;
    }

    @Override
    public void update()
    {
        telemetry.addData("pose est: ", cameraUpdate.pose);
        telemetry.addData("pose conf: ", cameraUpdate.confidence);
    }
}
