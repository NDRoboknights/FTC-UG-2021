package org.firstinspires.ftc.teamcode.util;

public class LocalizationUtility {

    //Method that finds the width of the field (left to right)
    public double getFieldWidth(double d1, double d2, double robotWidth){
        return d1 + d2 + robotWidth;
    }

    //Method that finds the length of the field (top to bottom)
    public double getFieldLength(double d3, double d4, double robotLength){
        return d3 + d4 + robotLength;
    }

    //Method that finds the x-coordinate of the robot
    public double getX(double d1, double d2, double fieldWidth){
        return (fieldWidth * d1) / (d1 + d2);
    }

    //Method that finds the y-coordinate of the robot
    public double getY(double d3, double d4, double fieldLength){
        return  (fieldLength * d3) / (d3 + d4);

    }
}