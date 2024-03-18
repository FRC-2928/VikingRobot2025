// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class ReadyShooter extends Command {
  /** Creates a new RaiseShooter. */
  public ReadyShooter() {
    this.addRequirements(Robot.cont.shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 
    
  }

  @Override
	public void execute() {
    Robot.cont.shooter.io.rotate(Constants.Shooter.readyShootRear); 
  }  

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { 
    return Math.abs(Robot.cont.shooter.inputs.angle.in(Units.Degrees) - Constants.Shooter.readyShootRear.in(Units.Degrees)) < 5;
  }
}
