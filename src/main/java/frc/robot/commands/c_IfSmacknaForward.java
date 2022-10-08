// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExampleSubsystem;

public class c_IfSmacknaForward extends CommandBase {
  private final ExampleSubsystem exampleSubsystem;

  /** Creates a new c_Forward. */
  public c_IfSmacknaForward(ExampleSubsystem es) {
    // Use addRequirements() here to declare subsystem dependencies.
    exampleSubsystem = es;
    addRequirements(exampleSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (exampleSubsystem.detectSmakna() == true) {
      exampleSubsystem.Forward();

    } else {
      exampleSubsystem.Stop();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    exampleSubsystem.Stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
