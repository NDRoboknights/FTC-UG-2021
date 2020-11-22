package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class REVHubIMU
{
	private BNO055IMU imu;

	public REVHubIMU(HardwareMap hMap, String name, BNO055IMU imu)
	{
		this.imu = imu;
		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
		parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
		parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
		parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
		parameters.loggingEnabled      = true;
		parameters.loggingTag          = "IMU";
		parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

		this.imu = hMap.get(BNO055IMU.class, name);
		this.imu.initialize(parameters);
	}

	public double getZAxisValue()
	{
		return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
	}
	public double getYAxisValue()
	{
		return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle;
	}
	public double getXAxisValue()
	{
		return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle;
	}

	public double normalizeValue(double value)
	{
		while(value >= 360) {
			value -= 360;
		}
		while(value < 0) {
			value += 360;
		}
		return value;
	}

	public double normalizeError(double error)
	{
		if(error < -180) {
			error += 360;
		}
		else if(error > 180) {
			error -= 360;
		}
		return error;
	}
}