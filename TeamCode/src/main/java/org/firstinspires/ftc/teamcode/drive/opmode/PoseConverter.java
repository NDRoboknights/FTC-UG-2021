package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;


public class PoseConverter
{
	public Pose2d convertGeometry(Translation2d translation2d, Rotation2d rotation2d)
	{
		return new Pose2d(translation2d.getX(), translation2d.getY(), rotation2d.getDegrees());
	}
}
