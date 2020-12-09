package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;


public class PoseConverter
{
	public static Pose2d convertToRoadRunner(Translation2d translation2d, Rotation2d rotation2d)
	{
		return new Pose2d(translation2d.getX(), translation2d.getY(), rotation2d.getDegrees());
	}

	public static com.arcrobotics.ftclib.geometry.Pose2d convertToFTCLib(Pose2d pose2d)
	{
		return new com.arcrobotics.ftclib.geometry.Pose2d(new Translation2d(pose2d.getX(), pose2d.getY()), new Rotation2d(pose2d.getHeading()));
	}
}