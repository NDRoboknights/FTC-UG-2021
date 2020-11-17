package org.firstinspires.ftc.teamcode.RoadRunner;

import com.acmerobotics.roadrunner.kinematics.MecanumKinematics;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;

public class MecanumBotConstraints
{
	public static final double NEW_MECANUM_WHEEL_DIAMETER = 96 / 25.4;
	public static final double COUNTS_PER_REVOLUTION_YELLOWJACKET_312 = 537.6;
	public static final double EXPECTED_RPM_YELLOWJACKET_312 = 312;
	public static final double OLD_MECANUM_WHEEL_DIAMETER = 100 / 25.4;

	double maxVelocity, maxAccel, maxJerk, maxAngularVelocity, maxAngularAcceleration, maxAngularJerk;
	public MecanumBotConstraints(double maxVelocity, double maxAccel, double maxJerk, double maxAngularVelocity, double maxAngularAcceleration, double maxAngularJerk){
		this.maxVelocity =  maxVelocity;
		this.maxAccel = maxAccel;
		this.maxJerk = maxJerk;
		this.maxAngularVelocity = maxAngularVelocity;
		this.maxAngularAcceleration = maxAngularAcceleration;
		this.maxAngularJerk = maxAngularJerk;
	}

	public MecanumConstraints mecanumConstraints = new MecanumConstraints(
			new DriveConstraints(maxVelocity, maxAccel, maxJerk, maxAngularVelocity, maxAngularAcceleration, maxAngularJerk),
			407 / 25.4, 336 / 25.4, 1);
}
