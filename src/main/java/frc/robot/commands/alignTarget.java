// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.vision.Limelight;
import frc.robot.subsystems.Drivetrain;

/** Add your docs here. */
public class alignTarget extends Command {
    private static Limelight limelight;
    private final Drivetrain drivetrain;
    private static final PIDController targetVerticalController = Constants.Drivetrain.targetVerticalControllerPID
        .createController();
    private static final PIDController targetHorizontalController = Constants.Drivetrain.targetHorizontalControllerPID
        .createController();

    public alignTarget(Drivetrain d) {
        drivetrain = d;
        limelight = this.drivetrain.limelight;

    }

    @Override
    public void execute() {
        Translation2d translation = getAlignTarget();
        drivetrain.drive(translation.getX(), translation.getY(), 0, false);
    }

    public Translation2d getAlignTarget() {
        limelight.setPipeline(1);
        if(limelight.hasValidTargets()) {
            Pose3d limePos = limelight.getRobotTagPose3d();
            double measurement = limePos.getX();
            double lateral = targetVerticalController.calculate(measurement, 0);
            double measurementY = -limePos.getZ();

            // xSpeed should end up between -1 and 1
            Logger.recordOutput("Align lateral Speed", lateral);
            Logger.recordOutput("Align lateral Speed", limelight.getTargetVerticalOffset());
            lateral = MathUtil.clamp(lateral, -1, 1); // Make it a unit value?
            double axialSpeed = targetHorizontalController.calculate(measurementY, -1.5);
            axialSpeed = MathUtil.clamp(axialSpeed, -1, 1);
            SmartDashboard.putNumber("JoystickDrive/Target align ySpeed", axialSpeed);

            double vLateralMetersPerSecond = MathUtil
                .applyDeadband(lateral * Constants.Drivetrain.maxVelocityMetersPerSec, 0.2);
            double vAxialMetersPerSecond = MathUtil
                .applyDeadband(axialSpeed * Constants.Drivetrain.maxVelocityMetersPerSec, 0.2);
            SmartDashboard.putNumber("JoystickDrive/Align Target X", vLateralMetersPerSecond);
            SmartDashboard.putNumber("JoystickDrive/Align Target Y", vAxialMetersPerSecond);

            return new Translation2d(vAxialMetersPerSecond, vLateralMetersPerSecond);
        }
        return new Translation2d(0, 0);
    }
}
