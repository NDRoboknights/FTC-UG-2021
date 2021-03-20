package org.firstinspires.ftc.teamcode.Auton.Localizer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.roboknights4348.lib.wpimath.src.main.java.edu.wpi.first.wpilibj.estimator.MecanumDrivePoseEstimator;
import com.roboknights4348.lib.wpimath.src.main.java.edu.wpi.first.wpilibj.util.Units;
import com.roboknights4348.lib.wpimath.src.main.java.edu.wpi.first.wpiutil.math.MatBuilder;
import com.roboknights4348.lib.wpimath.src.main.java.edu.wpi.first.wpiutil.math.Nat;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.Robot.MecanumBot;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

public class SensorFusionLocalizer implements Localizer
{
    MecanumBot bot;
    HardwareMap hardwareMap;

    MecanumDrivePoseEstimator poseEstimator = new MecanumDrivePoseEstimator(
            new Rotation2d(0.0),
            new com.arcrobotics.ftclib.geometry.Pose2d(new Translation2d(Units.feetToMeters(2), 0.168), new Rotation2d(0)),
            bot.mecanumDriveKinematics,
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01),
            new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.02),
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.02)
    );

    T265Camera slamera = new T265Camera(MecanumBot.cameraRelativePosition, MecanumBot.encoderMeasurementCovariance, hardwareMap.appContext);

    public SensorFusionLocalizer(HardwareMap hardwareMap)
    {
        this.hardwareMap = hardwareMap;
        bot = new MecanumBot(hardwareMap);
    }

    @NotNull
    @Override
    public Pose2d getPoseEstimate()
    {
        com.arcrobotics.ftclib.geometry.Pose2d p2d = poseEstimator.getEstimatedPosition();
        return new Pose2d(p2d.getX(), p2d.getY(), p2d.getHeading());
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d)
    {
        poseEstimator.resetPosition(new com.arcrobotics.ftclib.geometry.Pose2d(
                new Translation2d(Units.feetToMeters(pose2d.getX()),
                        Units.feetToMeters(pose2d.getY())), new Rotation2d()),
                new Rotation2d(pose2d.getHeading()));
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

    }
}
