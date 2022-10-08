// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExampleSubsystem;

public class c_smackago extends CommandBase {
  ExampleSubsystem es;

  /** Creates a new c_smackago. */
  public c_smackago(ExampleSubsystem exampleSubsystem) {
    es = exampleSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(es);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    es.Forward();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    es.Stop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return es.detectSmakna();
  }
}
