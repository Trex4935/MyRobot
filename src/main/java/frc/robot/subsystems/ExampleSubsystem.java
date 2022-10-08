// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {

  WPI_TalonFX motor1;
  WPI_TalonFX motor2;
  WPI_TalonFX motor3;

  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {
    motor1 = new WPI_TalonFX(1);
    motor2 = new WPI_TalonFX(2);
    motor3 = new WPI_TalonFX(3);
  }

  // Moves the robot forwards
  public void Forward() {
    motor1.set(0.5);
    motor2.set(0.5);
  }

  // Moves the robot backwards
  public void Backwards() {
    motor1.set(-0.5);
    motor2.set(-0.5);
  }

  // Stops the Robot
  public void Stop() {
    motor1.stopMotor();
    motor2.stopMotor();
    motor3.stopMotor();
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
