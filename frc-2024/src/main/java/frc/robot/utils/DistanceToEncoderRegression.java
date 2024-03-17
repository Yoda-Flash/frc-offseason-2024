package frc.robot.utils;

public class DistanceToEncoderRegression {

    public static double getWristPosition(double distance){
        return -0.0006*distance - 0.1123;
    //return m * distance + b;
        // return distance;
    }
}