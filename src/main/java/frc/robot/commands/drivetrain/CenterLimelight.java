// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CenterLimelight extends Command {
  /** Creates a new centerLimelight. */
  private Angle offsetX;
  private Angle offsetY;
  private double xSpeed;
  private double ySpeed;
  public CenterLimelight() {
    // Use addRequirements() here to declare subsystem dependencies.
    this.addRequirements(Robot.cont.drivetrain);
    this.offsetX = Units.Rotations.of(0);
    this.offsetY = Units.Rotations.of(0);
  }

  public CenterLimelight(Angle offsetX, Angle offsetY) {
    this.addRequirements(Robot.cont.drivetrain);
    this.offsetX = offsetX;
    this.offsetY = offsetY;      
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  @Override
  public void execute() {
    xSpeed = (13.302-Robot.cont.drivetrain.limelightNote.getBluePose3d().getX())*2;
    ySpeed = (2.658-Robot.cont.drivetrain.limelightNote.getBluePose3d().getY())*2;
    if(Robot.cont.drivetrain.limelightNote.hasValidTargets()){
      Robot.cont.drivetrain
          .control(
            Robot.cont.drivetrain
              .rod(
                new ChassisSpeeds(
                  xSpeed,
                  ySpeed,
                  0
                )
              )
      );
    }
      Logger.recordOutput("Drivetrain/Auto/XSpeed", xSpeed);
      Logger.recordOutput("Drivetrain/Auto/YSpeed",  ySpeed);
      Logger.recordOutput("Drivetrain/Auto/Center Is Finished", false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      Robot.cont.drivetrain.halt();
      Logger.recordOutput("Drivetrain/Auto/Center Is Finished", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(13.302-Robot.cont.drivetrain.limelightNote.getBotPose2d().getX()) < 0.01) && (Math.abs(2.658-Robot.cont.drivetrain.limelightNote.getPose2d().getY()) < 0.01) || !Robot.cont.drivetrain.limelightNote.hasValidTargets();
  }
}
