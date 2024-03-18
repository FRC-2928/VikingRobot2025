// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Robot;

public class ReadyShooter extends InstantCommand {
  /** Creates a new RaiseShooter. */
  public ReadyShooter() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 
    Robot.cont.shooter.io.rotate(Constants.Shooter.readyShootRear); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { return true; }
}
