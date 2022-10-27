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
    // Motors;
    public static final int dtfrontleftmotorID = 11;
    public static final int dtfrontrightmotorID = 22;
    public static final int dtbackleftmotorID = 33;
    public static final int dtbackrightmotorID = 44;
    // Settings
    public static final double dtMaxSpeed = 1;
    public static final double encoderTicksPerTurn = 2048;

}
