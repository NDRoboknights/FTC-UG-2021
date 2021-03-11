package org.firstinspires.ftc.teamcode.fusion;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.roboknights4348.lib.wpimath.src.main.java.edu.wpi.first.wpilibj.estimator.MecanumDrivePoseEstimator;
import com.roboknights4348.lib.wpimath.src.main.java.edu.wpi.first.wpilibj.kinematics.MecanumDriveKinematics;

public class FusionTest extends LinearOpMode {
    MecanumDrivePoseEstimator poseEstimator;
    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

    }
}
