package org.firstinspires.ftc.teamcode.fusion;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.roboknights4348.lib.wpimath.src.main.java.edu.wpi.first.wpilibj.estimator.MecanumDrivePoseEstimator;
import com.roboknights4348.lib.wpimath.src.main.java.edu.wpi.first.wpilibj.util.Units;
import com.roboknights4348.lib.wpimath.src.main.java.edu.wpi.first.wpiutil.math.MatBuilder;
import com.roboknights4348.lib.wpimath.src.main.java.edu.wpi.first.wpiutil.math.Nat;

import org.firstinspires.ftc.teamcode.Robot.MecanumBot;

public class FusionTest extends LinearOpMode {

    MecanumBot bot = new MecanumBot(hardwareMap);
    MecanumDrivePoseEstimator poseEstimator = new MecanumDrivePoseEstimator(
            new Rotation2d(0.0),
            new Pose2d(new Translation2d(Units.feetToMeters(2), 0.168), new Rotation2d(0)),
            bot.mecanumDriveKinematics,
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01),
            new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.02),
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.02)
    );

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

    }
}
