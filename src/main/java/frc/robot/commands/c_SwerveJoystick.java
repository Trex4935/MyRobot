// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class c_SwerveJoystick extends CommandBase {

  private final Drivetrain dt;
  private final Supplier<Double> xSpdFunc, ySpdFunc, trnSpdFunc;
  private final Supplier<Boolean> fieldOrientedFunc;
  private final SlewRateLimiter xLimiter, yLimiter, trnLimiter;

  /** Creates a new c_SwerveJoystick. */
  public c_SwerveJoystick(Drivetrain dt,
    Supplier<Double> xSpdFunc, Supplier<Double> ySpdFunc, Supplier<Double> trnSpdFunc,
    Supplier<Boolean> fieldOrientedFunc) {
    this.dt = dt;
    this.xSpdFunc = xSpdFunc;
    this.ySpdFunc = ySpdFunc;
    this.trnSpdFunc = trnSpdFunc;
    this.fieldOrientedFunc = fieldOrientedFunc;
    this.xLimiter = new SlewRateLimiter(Constants.dtMaxAcceleration);
    this.yLimiter = new SlewRateLimiter(Constants.dtMaxAcceleration);
    this.trnLimiter = new SlewRateLimiter(Constants.dtMaxAngAcceleration);

    // Use addRequirements() here to declare subsystem dependencies.
    
    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Gets joystick inputs
    double xSpeed = xSpdFunc.get();
    double ySpeed = ySpdFunc.get();
    double turnSpeed = trnSpdFunc.get();

    //Applies deadband
    xSpeed = Math.abs(xSpeed) > Constants.deadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > Constants.deadband ? ySpeed : 0.0;
    turnSpeed = Math.abs(turnSpeed) > Constants.deadband ? turnSpeed : 0.0;

    //Smoothens driving
    xSpeed = xLimiter.calculate(xSpeed) * Constants.dtMaxSpeed;
    ySpeed = yLimiter.calculate(ySpeed) * Constants.dtMaxSpeed;
    turnSpeed = trnLimiter.calculate(turnSpeed) * Constants.dtMaxAngSpeed;

    //Construct desired chassis speeds
    ChassisSpeeds chassisSpeeds;
    if (fieldOrientedFunc.get()) {
      //Relative to field
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed, dt.getAngleRotation());
    }
    else {
      //Realative to robot
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);
    }

    //Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates = dt.swerveKin.toSwerveModuleStates(chassisSpeeds);

    //Output module states to wheels
    dt.setModuleStates(moduleStates);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
