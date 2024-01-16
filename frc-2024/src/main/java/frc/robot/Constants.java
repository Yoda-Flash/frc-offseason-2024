// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class ElevatorPivot {
    public static final int kDriverControllerPort = 0;
    public static final int kMotorID1 = 1;
    public static final int kMotorID2 = 2;
    public static final int kMotorID3 = 3;
    public static final int kMotorID4 = 4;
    public static final int kPivotGearRatio = 200;
    public static final double kNeoTicksPerRevolution = 42;
    public static final double elevatorPivot_kP = 0.5;
    public static final double elevatorPivot_kI = 0;
    public static final double elevatorPivot_kD = 0;
    public static final double kDeadBand = 0.05;
  }
}
