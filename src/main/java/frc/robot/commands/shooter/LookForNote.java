// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class LookForNote extends Command {
  /** Creates a new lookForNote. */
  public LookForNote(Measure<Angle> rotAmount, Boolean repeat) { 
    this.addRequirements(Robot.cont.drivetrain); 
    this.rotationAmount = rotAmount;
    this.repeat = repeat;
  }

  private Measure<Angle> initalAngle;
  private boolean hasTurned;
  private Measure<Angle> currentAngle;
  private final Boolean repeat;
  private final Measure<Angle> rotationAmount;
  private Measure<Angle> setpoint;
  private final ProfiledPIDController absoluteController = Constants.Drivetrain.absoluteRotationPID
		.createProfiledController(Constants.Drivetrain.absoluteRotationConstraints);
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // this.initalAngle = Robot.cont.drivetrain.gyroInputs.yawPosition;
    this.initalAngle = Units.Rotations.of(Robot.cont.drivetrain.est.getEstimatedPosition().getRotation().getRotations());
    this.hasTurned = false;
    this.absoluteController.enableContinuousInput(-0.5,0.5);
    this.setpoint = this.initalAngle.plus(this.rotationAmount);
    this.absoluteController.reset(this.initalAngle.in(Units.Rotations));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.currentAngle = Units.Rotations.of(Robot.cont.drivetrain.est.getEstimatedPosition().getRotation().getRotations()); 
    double measurement = this.currentAngle.in(Units.Rotations);
    double computedPidValue = -this.absoluteController.calculate(measurement,this.setpoint.in(Units.Rotations));
    double currentRotation = Math.abs(this.currentAngle.minus(this.initalAngle).in(Units.Radians));
    if(currentRotation >= (Math.abs(this.rotationAmount.in(Units.Radians))-(0.06)) && !this.hasTurned){
      this.setpoint = this.initalAngle.minus(this.rotationAmount);
      this.hasTurned = true;
    }
    Measure<Velocity<Angle>> rotationSpeed = Constants.Drivetrain.maxAngularVelocity.times( MathUtil.applyDeadband(computedPidValue,0.015));
    Robot.cont.drivetrain
      .control(
        Robot.cont.drivetrain
          .rod(
          new ChassisSpeeds(
            0,
            0,
            rotationSpeed.in(Units.RadiansPerSecond)
            )
          )
      );
    Logger.recordOutput("Drivetrain/Auto/setpoint",(this.setpoint.in(Units.Radians)));
    Logger.recordOutput("Drivetrain/Auto/currentAngle",(this.currentAngle.in(Units.Radians)));
    Logger.recordOutput("Drivetrain/Auto/initialAngle",(this.initalAngle.in(Units.Radians)));
    Logger.recordOutput("Drivetrain/Auto/PIDvalue", computedPidValue);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
      Robot.cont.drivetrain.halt();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean hasRotatedThruAngle;
    double amountRotated = Math.abs(this.currentAngle.minus(this.initalAngle).in(Units.Radians));
    if(this.repeat && this.rotationAmount.in(Units.Radians)>0){
      hasRotatedThruAngle = this.currentAngle.in(Units.Radians) <=  -this.rotationAmount.in(Units.Radians) + this.initalAngle.in(Units.Radians);
    }
    else if(this.repeat && this.rotationAmount.in(Units.Radians)<0){
      hasRotatedThruAngle = this.currentAngle.in(Units.Radians) >=  -this.rotationAmount.in(Units.Radians) + this.initalAngle.in(Units.Radians);
    }
    else{
      hasRotatedThruAngle = amountRotated >=  Math.abs(this.rotationAmount.in(Units.Radians));
    }
    Logger.recordOutput("Drivetrain/Auto/amountRotated",amountRotated);
    Logger.recordOutput("Drivetrain/Auto/hasRotatedThruAngle",hasRotatedThruAngle);
    return Robot.cont.drivetrain.limelightNote.hasValidTargets() || hasRotatedThruAngle;
  }
}
