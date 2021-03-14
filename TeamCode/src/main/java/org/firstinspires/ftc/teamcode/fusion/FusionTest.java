package org.firstinspires.ftc.teamcode.fusion;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.roboknights4348.lib.wpimath.src.main.java.edu.wpi.first.wpilibj.estimator.MecanumDrivePoseEstimator;
import com.roboknights4348.lib.wpimath.src.main.java.edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import com.roboknights4348.lib.wpimath.src.main.java.edu.wpi.first.wpilibj.util.Units;
import com.roboknights4348.lib.wpimath.src.main.java.edu.wpi.first.wpiutil.math.MatBuilder;
import com.roboknights4348.lib.wpimath.src.main.java.edu.wpi.first.wpiutil.math.Nat;
import com.spartronics4915.lib.T265Camera;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot.MecanumBot;
import org.firstinspires.ftc.teamcode.TeleOp.DrivetrainTest;

public class FusionTest extends OpMode {

    MecanumBot bot;

    DrivetrainTest drivetrainTest;

    MecanumDrivePoseEstimator poseEstimator = new MecanumDrivePoseEstimator(
            new Rotation2d(0.0),
            new Pose2d(new Translation2d(Units.feetToMeters(2), 0.168), new Rotation2d(0)),
            bot.mecanumDriveKinematics,
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01),
            new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.02),
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.02)
    );

    T265Camera slamera = new T265Camera(MecanumBot.cameraRelativePosition, MecanumBot.encoderMeasurementCovariance, hardwareMap.appContext);

    Pose2d startingPosition;

    @Override
    public void init()
    {
        bot = new MecanumBot(hardwareMap);
        bot.init(hardwareMap);

        drivetrainTest = new DrivetrainTest(bot, hardwareMap, gamepad1);
        drivetrainTest.init();

        startingPosition = new Pose2d(bot.xDist.getDistance(DistanceUnit.METER),
                bot.yDist.getDistance(DistanceUnit.METER),new Rotation2d());

        slamera.setPose(startingPosition);
        slamera.start();
        poseEstimator.update(new Rotation2d(Units.degreesToRadians(bot.imu.getZAxisValue())),
                bot.mecanumDriveKinematics.toWheelSpeeds(new ChassisSpeeds()));
    }

    public void loop()
    {
        T265Camera.CameraUpdate cameraUpdate = slamera.getLastReceivedCameraUpdate();
        telemetry.addData("vision pose (meters): ", cameraUpdate.pose);
        telemetry.addData("vision confidence: ", cameraUpdate.confidence);

        poseEstimator.updateWithTime(this.time, new Rotation2d(bot.imu.getZAxisValue()), bot.getWheelSpeeds());
        poseEstimator.addVisionMeasurement(cameraUpdate.pose, this.time);
        telemetry.addData("pose estimator pose (meters): ", poseEstimator.getEstimatedPosition());

        drivetrainTest.loop();
        telemetry.update();

    }

}
