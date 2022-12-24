// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

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

    // Controller

    // Axis
    public static final int rightAxisID = 5;
    public static final int leftAxisID = 1;

    // Motors
    public static final int motor1ID = 1;
    public static final int motor2ID = 2;
    public static final int motor3ID = 3;
    public static final int motor4ID = 4;
    

    // Drive Train
    //Motors Drive;
    public static final int dtfrontleftmotorID = 11;
    public static final int dtfrontrightmotorID = 22;
    public static final int dtbackleftmotorID = 33;
    public static final int dtbackrightmotorID = 44;

    //MotorTurn
    public static final int turnfrontleftmotorID = 16;
    public static final int turnfrontrightmotorID = 28;
    public static final int turnbackleftmotorID = 36;
    public static final int turnbackrightmotorID = 48;

    //Absolute Encoder Placeholder
    public static final int absoluteEncoderID = 12;

    //PID Controller-related
    public static final double kPTurning = 0.4;
    public static final double kPxController = 0.4;
    public static final double kPyController = 0.5;
    public static final double kPtheta = 0.4;

    //Constraints for profiled PID controller
    public static final TrapezoidProfile.Constraints thetaConstraints =
     new Constraints(Constants.dtMaxAngSpeed, Constants.dtMaxAngAcceleration);
    //Settings
    public static final double dtMaxSpeed = 1;
    //Placeholder max angular speed
    public static final double dtMaxAngSpeed = 2 * Math.PI;
    //Placeholder max acceleration
    public static final double dtMaxAcceleration = 1;
    //Placeholder max angular acceleration
    public static final double dtMaxAngAcceleration = 1;
    public static final double encoderTicksPerTurn = 2048;
    public static final double turnMotorGearRatio = 4;
    public static final double driveMotorGearRatio = 6;
    public static final double wheelDiameter = 6;
    public static final double wheelRadius = wheelDiameter / 2;
    //public static final double driveEncoderMeters = driveMotorGearRatio * wheelDiameter * Math.PI;
    public static final double turnEncoderRadians = turnMotorGearRatio * 2 * Math.PI;
    //public static final double driveEncoderRPMMetersPerSec = driveEncoderMeters / 60;
    public static final double turnEncoderRPMRadPerSec = turnEncoderRadians / 60;
    //Placeholder deadband
    public static final double deadband = 1;
    public static final Translation2d frontleftWheelPos = new Translation2d(0,0);
    public static final Translation2d frontrightWheelPos = new Translation2d(1,0);
    public static final Translation2d backleftWheelPos = new Translation2d(0,-1);
    public static final Translation2d backrightWheelPos = new Translation2d(1,-1);
    
    public static final SwerveDriveKinematics  driveKin = new SwerveDriveKinematics(
        frontleftWheelPos, frontrightWheelPos, backleftWheelPos, backrightWheelPos
    );


}