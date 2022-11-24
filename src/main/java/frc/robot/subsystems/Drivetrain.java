// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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



  public static AHRS ahrs;


  

  /** Creates a new Drivetrain. */
  public Drivetrain() {

    dtfrontleftmotor = new WPI_TalonFX(Constants.dtfrontleftmotorID);
    dtfrontrightmotor = new WPI_TalonFX(Constants.dtfrontrightmotorID);
    dtbackleftmotor = new WPI_TalonFX(Constants.dtbackleftmotorID);
    dtbackrightmotor = new WPI_TalonFX(Constants.dtbackrightmotorID);

    leftmotors = new MotorControllerGroup(dtfrontleftmotor, dtbackleftmotor);
    rightmotors = new MotorControllerGroup(dtfrontrightmotor, dtbackrightmotor);

    
    

    ahrs = new AHRS(SPI.Port.kMXP);
    ahrs.calibrate();
    ahrs.reset();

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

  /**
   * This methods stops all motors
   */
  public void stopMotors() {
   
  }

  // Gets angle from gyro and returns it
  public double getAngle() {
    double angle = ahrs.getAngle();
    return angle;
  }

/**
 * This method reset encoders
 */
  public void resetEncoder(){
    dtfrontleftmotor.setSelectedSensorPosition(0);
    dtfrontrightmotor.setSelectedSensorPosition(0);
    dtbackleftmotor.setSelectedSensorPosition(0);
    dtbackrightmotor.setSelectedSensorPosition(0);
  }
/**
 * This methods gives back encoder position.
 * @param selectedEncoder
 * @return value of selected encoder
 */
  public double getEncoderAngle( double selectedEncoder ){
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
    System.out.println(getAngle());
    System.out.println(getEncoderAngle(Constants.dtfrontleftmotorID));

  }
}
