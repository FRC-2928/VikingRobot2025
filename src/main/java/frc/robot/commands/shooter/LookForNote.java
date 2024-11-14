// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class LookForNote extends Command {
  /** Creates a new lookForNote. */
  public LookForNote() { this.addRequirements(Robot.cont.drivetrain); }

  private Measure<Angle> initalAngle;
  private boolean hasTurned;
  private Measure<Angle> currentAngle;
  private final static Measure<Angle> rotationAmount =  Units.Radians.of(Math.PI/3);
  private final ProfiledPIDController absoluteController = Constants.Drivetrain.absoluteRotationPID
		.createProfiledController(Constants.Drivetrain.absoluteRotationConstraints);
    

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // this.initalAngle = Robot.cont.drivetrain.gyroInputs.yawPosition;
    this.initalAngle = Units.Rotations.of(Robot.cont.drivetrain.est.getEstimatedPosition().getRotation().getRotations());
    this.hasTurned = false;
    // this.absoluteController.enableContinuousInput(-0.5,0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // this.currentAngle = Robot.cont.drivetrain.gyroInputs.yawPosition;
    this.currentAngle = Units.Rotations.of(Robot.cont.drivetrain.est.getEstimatedPosition().getRotation().getRotations());
    //If robot has turned less than 30 degrees counter-clockwise keep turning
    // if((this.currentAngle.minus(LookForNote.rotationAmount)).in(Units.Radians) <= this.initalAngle.in(Units.Radians) && !this.hasTurned) {
      Robot.cont.drivetrain
        .control(
          Robot.cont.drivetrain.joystickSpeeds
            .plus(
              Robot.cont.drivetrain
                .rod(
                  new ChassisSpeeds(
                    0,
                    0,
                    /* TODO: unhackify this mess, put things into local variables and use those instead of having everything in one place
                     * also there's no deadband here right now, we should add a deadband so that once it's close we consider it "good enough" and stop the control
                    */
                    /* Use pid to calculate rotation speed */
                    Constants.Drivetrain.maxAngularVelocity.times(
                    -this.absoluteController.calculate(this.currentAngle.in(Units.Rotations),/*this.initalAngle.in(Units.Rotations)*/ LookForNote.rotationAmount.in(Units.Rotations))
                    ).in(Units.RadiansPerSecond)

                  )
                )
            )
       
            );
      Logger.recordOutput("Drivetrain/Auto/setpoint",(/*this.initalAngle.in(Units.Radians)*/ LookForNote.rotationAmount.in(Units.Radians)));
      Logger.recordOutput("Drivetrain/Auto/currentAngle",(this.currentAngle.in(Units.Radians)));
      Logger.recordOutput("Drivetrain/Auto/initialAngle",(this.initalAngle.in(Units.Radians)));
    // } else {
    //   this.hasTurned = true;
    //   Robot.cont.drivetrain
    //     .control(
    //       Robot.cont.drivetrain.joystickSpeeds
    //         .plus(
    //           Robot.cont.drivetrain
    //             .rod(
    //               new ChassisSpeeds(
    //                 0,
    //                 0,
    //                 /* Use pid to calculate rotation speed */
    //                 -this.absoluteController.calculate(this.currentAngle.in(Units.Rotations),this.initalAngle.in(Units.Rotations) + LookForNote.rotationAmount.in(Units.Rotations))

    //               )
    //             )
    //         )
    //     );
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
      Robot.cont.drivetrain.halt();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Robot.cont.drivetrain.limelightNote.hasValidTargets() || this.currentAngle.in(Units.Radians) + (LookForNote.rotationAmount.in(Units.Radians)) <= this.initalAngle.in(Units.Radians);
  }
}
