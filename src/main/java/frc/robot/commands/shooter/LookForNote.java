// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class LookForNote extends Command {
  /** Creates a new lookForNote. */
  public LookForNote() { this.addRequirements(Robot.cont.drivetrain); }

  private double initalAngle;
  private boolean hasTurned;
  private final static double ROTATION_SPEED = 1;
  private double currentAngle;
  private final static double rotationAmount = Math.PI/3;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.initalAngle = Robot.cont.drivetrain.gyroInputs.yawPosition.in(Units.Radian);
    this.hasTurned = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.currentAngle = Robot.cont.drivetrain.gyroInputs.yawPosition.in(Units.Radian);
    //If robot has turned less than 30 degrees counter-clockwise keep turning
    if((this.currentAngle - (LookForNote.rotationAmount) <= this.initalAngle) && !this.hasTurned) {
      Robot.cont.drivetrain
        .control(
          Robot.cont.drivetrain.joystickSpeeds
            .plus(
              Robot.cont.drivetrain
                .rod(
                  new ChassisSpeeds(
                    0,
                    0,
                    -LookForNote.ROTATION_SPEED

                  )
                )
            )
        );
    } else {
      this.hasTurned = true;
      Robot.cont.drivetrain
        .control(
          Robot.cont.drivetrain.joystickSpeeds
            .plus(
              Robot.cont.drivetrain
                .rod(
                  new ChassisSpeeds(
                    0,
                    0,
                    LookForNote.ROTATION_SPEED

                  )
                )
            )
        );
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Robot.cont.drivetrain.limelightNote.hasValidTargets() || this.currentAngle + (LookForNote.rotationAmount) <= this.initalAngle;
  }
}
