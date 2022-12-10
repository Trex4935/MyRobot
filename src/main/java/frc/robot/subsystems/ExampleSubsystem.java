// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ExampleSubsystem extends SubsystemBase {

  WPI_TalonFX motor1;
  WPI_TalonFX motor2;
  WPI_TalonFX motor3;
  DigitalInput smacka2;
  WPI_TalonFX motor4;
  DigitalInput smacka1;
  
  MotorControllerGroup altleftmotors;
  MotorControllerGroup altrightmotors;

  DifferentialDrive altDiffDrive;

  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {
    motor1 = new WPI_TalonFX(Constants.motor1ID);
    motor2 = new WPI_TalonFX(Constants.motor2ID);
    motor3 = new WPI_TalonFX(Constants.motor3ID);
    motor4 = new WPI_TalonFX(Constants.motor4ID);
    smacka2 = new DigitalInput(2);
    smacka1 = new DigitalInput(1);

    altleftmotors = new MotorControllerGroup(motor1, motor3);
    altrightmotors = new MotorControllerGroup(motor2, motor4);

    altDiffDrive = new DifferentialDrive(altleftmotors, altrightmotors);
  }

  // Moves the robot forwards
  public void Forward() {
    motor1.set(0.5);
    motor2.set(0.5);
    motor3.set(0.5);
    motor4.set(0.5);
  }

  // Moves the robot forward at a slower speed
  public void SlowForward() {
    motor1.set(0.2);
    motor2.set(0.2);
    motor3.set(0.2);
    motor4.set(0.2);
  }

  // Moves the robot forward at a faster speed
  public void FastForward() {
    motor1.set(0.8);
    motor2.set(0.8);
    motor3.set(0.8);
    motor4.set(0.8);
  }

  // Moves the robot backwards
  public void Backwards() {
    motor1.set(-0.5);
    motor2.set(-0.5);
    motor3.set(-0.5);
    motor4.set(-0.5);
  }

  public boolean detectSmakna() {
    boolean smaknaTrue = smacka1.get();

    return !smaknaTrue;
  }

  // Stops the Robot
  public void Stop() {
    motor1.stopMotor();
    motor2.stopMotor();
    motor3.stopMotor();
    motor4.stopMotor();
  }

  public void driveWithJoystick(Joystick arduino) {

    altDiffDrive.tankDrive((arduino.getRawAxis(Constants.leftAxisID)) * Constants.dtMaxSpeed,
    (arduino.getRawAxis(Constants.rightAxisID))*Constants.dtMaxSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
