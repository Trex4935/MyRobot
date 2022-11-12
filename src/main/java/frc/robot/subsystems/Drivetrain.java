// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;
import frc.robot.extensions.SwerveModule;

public class Drivetrain extends SubsystemBase {
  WPI_TalonFX dtfrontleftmotor;
  WPI_TalonFX dtfrontrightmotor;
  WPI_TalonFX dtbackleftmotor;
  WPI_TalonFX dtbackrightmotor;

  WPI_TalonFX turnfrontleftmotor;
  WPI_TalonFX turnfrontrightmotor;
  WPI_TalonFX turnbackleftmotor;
  WPI_TalonFX turnbackrightmotor;

  MotorControllerGroup leftmotors;
  MotorControllerGroup rightmotors;

  DifferentialDrive diffDrive;

  public static AHRS ahrs;

  SwerveDriveOdometry swerveOdo;
  SwerveDriveKinematics swerveKin;

  SwerveModule swerveModuleLF;
  SwerveModule swerveModuleRF;
  SwerveModule swerveModuleLB;
  SwerveModule swerveModuleRB;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // Drive Motors
    dtfrontleftmotor = new WPI_TalonFX(Constants.dtfrontleftmotorID);
    dtfrontrightmotor = new WPI_TalonFX(Constants.dtfrontrightmotorID);
    dtbackleftmotor = new WPI_TalonFX(Constants.dtbackleftmotorID);
    dtbackrightmotor = new WPI_TalonFX(Constants.dtbackrightmotorID);
    // Turn Motors
    turnfrontleftmotor = new WPI_TalonFX(Constants.turnfrontleftmotorID);
    turnfrontrightmotor = new WPI_TalonFX(Constants.turnfrontrightmotorID);
    turnbackleftmotor = new WPI_TalonFX(Constants.turnbackleftmotorID);
    turnbackrightmotor = new WPI_TalonFX(Constants.turnbackrightmotorID);

    leftmotors = new MotorControllerGroup(dtfrontleftmotor, turnfrontleftmotor);
    rightmotors = new MotorControllerGroup(dtfrontrightmotor, dtbackrightmotor);

    diffDrive = new DifferentialDrive(leftmotors, rightmotors);

    ahrs = new AHRS(SPI.Port.kMXP);
    ahrs.calibrate();
    ahrs.reset();

    // Declare Swerve Module Class
    swerveModuleLF = new SwerveModule(dtfrontleftmotor, turnfrontleftmotor);

    // Declaring kinemathics, that means the wheel position on drive train
    swerveKin = new SwerveDriveKinematics(Constants.frontleftWheelPos, Constants.frontrightWheelPos,
        Constants.backleftWheelPos, Constants.backrightWheelPos);
    // Declaring Odometry, that means sensor value needed to updated swerve drive.
    swerveOdo = new SwerveDriveOdometry(swerveKin, getAngleRotation(), new Pose2d()); // Need to change Pose2D to actual
                                                                                      // position of robot on the field.
  }

  /**
   * Method that drives forward at same speed
   * 
   * @param speed
   */
  public void driveForward(int speed) {

    diffDrive.tankDrive(speed, speed);

  }

  /**
   * This method controls the robot using the joystick
   * 
   * @param controller
   */
  public void driveWithController(XboxController controller) {

    diffDrive.tankDrive((controller.getRawAxis(Constants.leftAxisID)) * Constants.dtMaxSpeed,
        (controller.getRawAxis(Constants.rightAxisID)) * Constants.dtMaxSpeed);
  }

  /**
   * This methods stops all motors
   */
  public void stopMotors() {
    leftmotors.set(0);
    rightmotors.set(0);
  }

  // Gets angle from gyro and returns it
  public double getAngle() {
    double angle = ahrs.getAngle();
    return angle;
  }

  // Gets angle from gyro and returns it
  public Rotation2d getAngleRotation() {
    double angle = ahrs.getAngle();
    return new Rotation2d(angle / (2 * 3.1416));
  }

  /**
   * This method reset encoders
   */
  public void resetEncoder() {
    dtfrontleftmotor.setSelectedSensorPosition(0);
    dtfrontrightmotor.setSelectedSensorPosition(0);
    dtbackleftmotor.setSelectedSensorPosition(0);
    dtbackrightmotor.setSelectedSensorPosition(0);
  }

  /**
   * This methods gives back encoder ticks position.
   * 
   * @param selectedEncoder
   * @return value of selected encoder
   */
  public double getEncoderTicks(double selectedEncoder) {
    if (selectedEncoder == Constants.dtfrontleftmotorID) {
      return dtfrontleftmotor.getSelectedSensorPosition();
    } else if (selectedEncoder == Constants.dtfrontleftmotorID) {
      return dtfrontrightmotor.getSelectedSensorPosition();
    } else if (selectedEncoder == Constants.dtfrontleftmotorID) {
      return dtbackleftmotor.getSelectedSensorPosition();
    } else if (selectedEncoder == Constants.dtfrontleftmotorID) {
      return dtbackrightmotor.getSelectedSensorPosition();
    } else {
      return 0;
    }

  }

  // Get Turn Encoder Angle

  // Get Drive Encoder Speed

  // Takes the rotation or internal ticks of Falcon Encoder and turn them to a
  // travel distance. gear ratio A:1 means, 1/A.
  public double ticksToAngle(double ticks, double gearRatio) {
    double nbTurnMotor = ticks / Constants.encoderTicksPerTurn;
    double nbTurnWheel = nbTurnMotor * gearRatio;
    double angleOfWheel = nbTurnWheel * 360;
    return angleOfWheel;

  }

  // Takes the rotation or internal ticks of Falcon Encoder and turn them to a
  // travel distance. gear ratio A:1 means, 1/A.
  public double ticksToPosition(double ticks, double wheelDiameter, double gearRatio) {
    double nbTurnMotor = ticks / Constants.encoderTicksPerTurn;
    double nbTurnWheel = nbTurnMotor * gearRatio;
    double distanceTravel = nbTurnWheel * Math.PI * wheelDiameter;
    return distanceTravel;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println(getAngle());
    // Give encoder Ticks for left front motor
    System.out.println(getEncoderTicks(Constants.dtfrontleftmotorID));
    System.out.println(swerveModuleLF.getDriveEncoderTicks());
    // Give encoder Angle for left front motor
    System.out.println(ticksToAngle(getEncoderTicks(Constants.turnfrontleftmotorID), Constants.turnMotorGearRatio));
    System.out.println(swerveModuleLF.getAngle());
    // Give encoder Distance for left front motor
    System.out.println(ticksToPosition(getEncoderTicks(Constants.dtfrontleftmotorID), Constants.wheelDiameter,
        Constants.driveMotorGearRatio));
    System.out.println(swerveModuleLF.getDistance());
    // Get State
    System.out.println(swerveModuleLF.getModuleState());
    System.out.println(swerveModuleLF.getSpeed(swerveModuleLF.getDriveEncoderVelocityTicks(), Constants.wheelDiameter));

  }
}
