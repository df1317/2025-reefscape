// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // Can ID constants

    // Drive Motor IDs
    public static final int driveMotorFR = 12;
    public static final int driveMotorFL = 13;
    public static final int driveMotorBR = 14;
    public static final int driveMotorBL = 15;

    // Turn motor IDs
    public static final int turnMotorFR = 16;
    public static final int turnMotorFL = 17;
    public static final int turnMotorBR = 18;
    public static final int turnMotorBL = 19;

    // CanCoder IDs
    public static final int canCoderFR = 20;
    public static final int canCoderRL = 21;
    public static final int canCoderBR = 22;
    public static final int canCoderBL = 23;

    // Scoring Motor IDs
    public static final int scoreMotor1 = 24;
    public static final int scoreMotor2 = 25;

    // Elevator Motor IDs
    public static final int elevatorMotorL = 26;
    public static final int elevatorMotorR = 27;

    // Sensor ports
    public static final int coralSensorPort = 1;
    public static final int elevatorEncoderL = 2;
    public static final int elevatorEncoderR = 3;

}
