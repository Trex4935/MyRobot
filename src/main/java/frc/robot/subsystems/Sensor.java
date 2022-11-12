// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Sensor extends SubsystemBase {
  public static boolean toggle1;
  /** Creates a new Sensor. */
  public Sensor() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    toggle1=RobotContainer.arduino.getRawButton(3);
  }
}
