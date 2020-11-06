package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.kinematics.TankKinematics;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SixWDBot extends Bot{

    //drivetrain motors
    public MotorEx lDrive1; //left drive motor 1
    public MotorEx lDrive2; //left drive motor 2
    public MotorEx rDrive1; //right drive motor 1
    public MotorEx rDrive2; //right drive motor 2

    //class constructor so all hardware is initialized when
    // SixWDBot bot = new SixWDBot(HardwareMap); is written
    public SixWDBot(HardwareMap hMap, Motor.ZeroPowerBehavior zeroPowerBehavior)
    {
        init(hMap);
        setZPB(zeroPowerBehavior);
    }

    //Same as above without ZPB in the signature, ZPB then defaults to brake.
    public SixWDBot(HardwareMap hMap)
    {
        init(hMap);
        setZPB(Motor.ZeroPowerBehavior.BRAKE);
    }
    //init function that initializes and maps hardware
    public void init(HardwareMap hardwareMap)
    {
        lDrive1 = new MotorEx(hardwareMap, "lMotor1", Motor.GoBILDA.RPM_312); //maps 1st lDrive motor to id "lMotor1"
        lDrive2 = new MotorEx(hardwareMap, "lMotor2", Motor.GoBILDA.RPM_312); //maps 2nd lDrive motor to id "lMotor2"
        rDrive1 = new MotorEx(hardwareMap, "rMotor1", Motor.GoBILDA.RPM_312); //maps 1st rDrive motor to id "rMotor1"
        rDrive2 = new MotorEx(hardwareMap, "rMotor2", Motor.GoBILDA.RPM_312); //maps 2nd rDrive motor to id "rMotor2"

        //inverts right motor
        rDrive1.setInverted(true);
        rDrive2.setInverted(true);
    }

    public void setZPB(Motor.ZeroPowerBehavior zpb)
    {
        lDrive1.setZeroPowerBehavior(zpb);
        lDrive2.setZeroPowerBehavior(zpb);
        rDrive1.setZeroPowerBehavior(zpb);
        rDrive2.setZeroPowerBehavior(zpb);
    }

    public void setLDrivePower(double power)
    {
        lDrive1.set(power);
        lDrive2.set(power);
    }

    public void setRDrivePower(double power)
    {
        rDrive1.set(power);
        rDrive1.set(power);
    }
}