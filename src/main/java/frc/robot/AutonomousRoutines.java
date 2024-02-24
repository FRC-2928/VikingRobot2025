package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.drivetrain.LockWheels;
import frc.robot.subsystems.Drivetrain;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

public final class AutonomousRoutines {
	public static SendableChooser<Command> createAutonomousChooser(final Drivetrain drivetrain) {
		final SendableChooser<Command> chooser = new SendableChooser<>();

		chooser
			.addOption(
				"Drive test trajectory",
				new SequentialCommandGroup(AutonomousRoutines.runTrajectory("test", drivetrain))
			);

		// Set up SysId routines
		chooser
			.addOption(
				"Drive SysId (Quasistatic Forward)",
				drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
			);
		chooser
			.addOption(
				"Drive SysId (Quasistatic Reverse)",
				drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
			);
		chooser.addOption("Drive SysId (Dynamic Forward)", drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
		chooser.addOption("Drive SysId (Dynamic Reverse)", drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));

		return chooser;
	}

	public static Command runTrajectory(final String trajectoryName, final Drivetrain drivetrain) {
		final ChoreoTrajectory traj = Choreo.getTrajectory(trajectoryName);
		SmartDashboard.putString("Choreo/Trajectory Name", trajectoryName);
		SmartDashboard.putNumber("Choreo/x0", traj.getInitialPose().getX());
		SmartDashboard.putNumber("Choreo/y0", traj.getInitialPose().getY());
		SmartDashboard.putNumber("Choreo/r0", traj.getInitialPose().getRotation().getDegrees());

		final var thetaController = new PIDController(0.5, 0, 0);
		thetaController.enableContinuousInput(-Math.PI, Math.PI);

		final Command swerveCommand = Choreo
			.choreoSwerveCommand(
				traj, // Choreo trajectory from above
				drivetrain.pose::getPoseMeters, // A function that returns the current field-relative pose of the robot:
				// your wheel or vision odometry
				new PIDController(0.1, 0.0, 0.0), // PIDController for field-relative X
				// translation (input: X error in meters,// output: m/s).
				new PIDController(0.1, 0.0, 0.0), // PIDController for field-relative Y
				// translation (input: Y error in meters, output: m/s).
				thetaController, // PID constants to correct for rotation error
				drivetrain::control,
				() -> DriverStation.getAlliance().get() == Alliance.Red, // Whether or not to mirror the path based on alliance (this assumes the path is created for the blue alliance)
				drivetrain // The subsystem(s) to require, typically your drive subsystem only
			);

		return Commands
			.sequence(
				Commands.runOnce(() -> drivetrain.resetOdometry(traj.getInitialPose())),
				swerveCommand,
				new LockWheels()
			);
	}
}
