// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  WPI_TalonFX dtfrontleftmotor;
  WPI_TalonFX dtfrontrightmotor;
  WPI_TalonFX dtbackleftmotor;
  WPI_TalonFX dtbackrightmotor;

  MotorControllerGroup leftmotors;
  MotorControllerGroup rightmotors;

  DifferentialDrive diffDrive;

  /** Creates a new Drivetrain. */
  public Drivetrain() {

    dtfrontleftmotor = new WPI_TalonFX(Constants.dtfrontleftmotorID);
    dtfrontrightmotor = new WPI_TalonFX(Constants.dtfrontrightmotorID);
    dtbackleftmotor = new WPI_TalonFX(Constants.dtbackleftmotorID);
    dtbackrightmotor = new WPI_TalonFX(Constants.dtbackrightmotorID);

    leftmotors = new MotorControllerGroup(dtfrontleftmotor, dtbackleftmotor);
    rightmotors = new MotorControllerGroup(dtfrontrightmotor, dtbackrightmotor);

    diffDrive = new DifferentialDrive(leftmotors,rightmotors);
    



  }

/**
 * Method that drives forward at same speed
 * @param speed
 */  
public void driveForward(int speed){

  diffDrive.tankDrive(speed, speed);

}
/**
 * This method controls the robot using the joystick
 * @param controller
 */
public void driveWithController(XboxController controller){


  diffDrive.tankDrive((controller.getRawAxis(Constants.leftAxisID))*Constants.dtMaxSpeed, (controller.getRawAxis(Constants.rightAxisID))*Constants.dtMaxSpeed);
}

/**
 * This methods stops all motors
 */
public void stopMotors(){
  leftmotors.set(0);
  rightmotors.set(0);
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
