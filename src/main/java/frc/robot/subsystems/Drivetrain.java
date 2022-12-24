// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.extensions.SwerveModule;

public class Drivetrain extends SubsystemBase {
  WPI_TalonSRX dtfrontleftmotor;
  WPI_TalonSRX dtfrontrightmotor;
  WPI_TalonSRX dtbackleftmotor;
  WPI_TalonSRX dtbackrightmotor;

  CANSparkMax turnfrontleftmotor;
  CANSparkMax turnfrontrightmotor;
  CANSparkMax turnbackleftmotor;
  CANSparkMax turnbackrightmotor;

  MotorControllerGroup leftmotors;
  MotorControllerGroup rightmotors;

  AnalogInput absoluteEncoder;

  public static AHRS ahrs;

  SwerveDriveOdometry swerveOdo;
  public SwerveDriveKinematics swerveKin;

  SwerveModule swerveModuleLF;
  SwerveModule swerveModuleRF;
  SwerveModule swerveModuleLB;
  SwerveModule swerveModuleRB;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // Drive Motors
    dtfrontleftmotor = new WPI_TalonSRX(Constants.dtfrontleftmotorID);
    dtfrontrightmotor = new WPI_TalonSRX(Constants.dtfrontrightmotorID);
    dtbackleftmotor = new WPI_TalonSRX(Constants.dtbackleftmotorID);
    dtbackrightmotor = new WPI_TalonSRX(Constants.dtbackrightmotorID);
    // Turn Motors
    turnfrontleftmotor = new CANSparkMax(Constants.turnfrontleftmotorID, MotorType.kBrushless);
    turnfrontrightmotor = new CANSparkMax(Constants.turnfrontrightmotorID, MotorType.kBrushless);
    turnbackleftmotor = new CANSparkMax(Constants.turnbackleftmotorID, MotorType.kBrushless);
    turnbackrightmotor = new CANSparkMax(Constants.turnbackrightmotorID, MotorType.kBrushless);

    leftmotors = new MotorControllerGroup(dtfrontleftmotor, dtbackleftmotor);
    rightmotors = new MotorControllerGroup(dtfrontrightmotor, dtbackrightmotor);

    absoluteEncoder = new AnalogInput(Constants.absoluteEncoderID);

    // Gyroscope
    ahrs = new AHRS(SPI.Port.kMXP);
    ahrs.calibrate();
    //ahrs.reset();

    new Thread(() -> {
      try {
        Thread.sleep(1000);
        ahrs.reset();
      } catch (Exception e) {
      }
    }).start();


    // Declare Swerve Module Class
    swerveModuleLF = new SwerveModule(dtfrontleftmotor, turnfrontleftmotor, false, false, absoluteEncoder, 0, false);
    swerveModuleLB = new SwerveModule(dtbackleftmotor, turnbackleftmotor, false, false, absoluteEncoder, 0, false);
    swerveModuleRF = new SwerveModule(dtfrontrightmotor, turnfrontrightmotor, false, false, absoluteEncoder, 0, false);
    swerveModuleRB = new SwerveModule(dtbackrightmotor, turnbackrightmotor, false, false, absoluteEncoder, 0, false);

    // Declaring kinematics, that means the wheel position on drive train
    swerveKin = new SwerveDriveKinematics(Constants.frontleftWheelPos, Constants.frontrightWheelPos,
        Constants.backleftWheelPos, Constants.backrightWheelPos);
    // Declaring Odometry, that means sensor value needed to updated swerve drive.
    swerveOdo = new SwerveDriveOdometry(swerveKin, getAngleRotation(), new Pose2d()); // Need to change Pose2D to actual
                                                                                      // position of robot on the field.
  }

  // ChassisSpeeds constructor (WIP)
  public ChassisSpeeds chsSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, getAngleRotation());

  // Conversion
  SwerveModuleState[] moduleStates = swerveKin.toSwerveModuleStates(chsSpeed);

  public void setModuleStates(SwerveModuleState[] desiredStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.dtMaxSpeed);
    swerveModuleLF.setDesiredState(desiredStates[0]);
    swerveModuleRF.setDesiredState(desiredStates[1]);
    swerveModuleLB.setDesiredState(desiredStates[2]);
    swerveModuleRB.setDesiredState(desiredStates[3]);
  }

  /**
   * Method that drives forward at same speed
   * 
   * @param speed
   */
  public void driveForward(int speed) {
    dtfrontleftmotor.set(0.5);
    dtfrontrightmotor.set(0.5);
    dtbackleftmotor.set(0.5);
    dtbackrightmotor.set(0.5);

  }

  /**
   * This method controls the robot using the joystick
   * 
   * @param controller
   */
  public void driveWithController(XboxController controller) {

  }

  //Stops the modules
  public void stopModules() {
    swerveModuleLF.stop();
    swerveModuleRF.stop();
    swerveModuleLB.stop();
    swerveModuleRB.stop();
  }

  /**
   * This methods stops all motors
   */
  public void stopMotors() {

  }

  //Resets gyro
  public void zeroHeading(){
    ahrs.reset();
  }

  // Gets angle from gyro and returns it
  public double getAngle() {
    double angle = ahrs.getAngle();
    return angle;
  }

  /* Gyro angle method used in https://www.youtube.com/watch?v=0Xi9yb1IMyA :
   * public double getHeading() {
    return Math.IEEEremainder(ahrs.getAngle(), 360);
  }
  */

  // Gets angle from gyro and returns it
  public Rotation2d getAngleRotation() {
    double angle = ahrs.getAngle();
    return new Rotation2d(angle / (2 * Math.PI));
  }

    /* Rotation2d converter used in https://www.youtube.com/watch?v=0Xi9yb1IMyA :
   * public double getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }
  */

  public Pose2d getPose() {
    return swerveOdo.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose){
    swerveOdo.resetPosition(pose, getAngleRotation());
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
    } else if (selectedEncoder == Constants.dtfrontrightmotorID) {
      return dtfrontrightmotor.getSelectedSensorPosition();
    } else if (selectedEncoder == Constants.dtbackleftmotorID) {
      return dtbackleftmotor.getSelectedSensorPosition();
    } else if (selectedEncoder == Constants.dtbackrightmotorID) {
      return dtbackrightmotor.getSelectedSensorPosition();
    } else {
      return 0;
    }

  }

  // Get Turn Encoder Angle

  // Get Drive Encoder Speed

  // Takes the rotation or internal ticks of Falcon Encoder and turn them to a
  // travel distance. gear ratio A:1 means, 1/A.
  public double ticksToAngle(double ticks) {
    double nbTurnMotor = ticks / Constants.encoderTicksPerTurn;
    double nbTurnWheel = nbTurnMotor * Constants.turnMotorGearRatio;
    double angleOfWheel = nbTurnWheel * 360;
    return angleOfWheel;

  }

  // Takes the rotation or internal ticks of Falcon Encoder and turn them to a
  // travel distance. gear ratio A:1 means, 1/A.
  public double ticksToPosition(double ticks) {
    double nbTurnMotor = ticks / Constants.encoderTicksPerTurn;
    double nbTurnWheel = nbTurnMotor * Constants.turnMotorGearRatio;
    double distanceTravel = nbTurnWheel * Math.PI * Constants.wheelDiameter;
    return distanceTravel;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    swerveOdo.update(getAngleRotation(),
    swerveModuleLF.getModuleState(), swerveModuleRF.getModuleState(),
    swerveModuleLB.getModuleState(), swerveModuleRB.getModuleState());

    System.out.println(getAngle());
    /*Used in video
    SmartDashboard.putNumber("Robot Heading", getAngle());
    */
    System.out.println("Robot Location: " + getPose().getTranslation().toString());
    /*Used in video
     *SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
     */
    // Give encoder Ticks for left front motor
    System.out.println(getEncoderTicks(Constants.dtfrontleftmotorID));
    System.out.println(swerveModuleLF.getDriveEncoderTicks());
    // Give encoder Angle for left front motor
    System.out.println(ticksToAngle(getEncoderTicks(Constants.turnfrontleftmotorID)));
    System.out.println(swerveModuleLF.getAngle());
    // Give encoder Distance for left front motor
    System.out.println(ticksToPosition(getEncoderTicks(Constants.dtfrontleftmotorID)));
    System.out.println(swerveModuleLF.getDistance());
    // Get State
    System.out.println(swerveModuleLF.getModuleState());

  }
}
