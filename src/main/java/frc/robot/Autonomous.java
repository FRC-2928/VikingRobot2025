package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.choreo.lib.*;
import com.pathplanner.lib.auto.*;
import com.pathplanner.lib.path.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.drivetrain.DriveTime;
import frc.robot.commands.shooter.*;

public final class Autonomous {
	public static SendableChooser<Command> createAutonomousChooser() {
		final SendableChooser<Command> chooser = new SendableChooser<>();

		chooser
			.addOption(
				"[comp] Five Note",
				new SequentialCommandGroup(
					new ReadyShooter(Constants.Shooter.readyShootRear, true),
					Autonomous.setInitialPose("MiddleFiveNote.1"),
					new ShootSpeaker(false, 2),
					Autonomous
						.path("MiddleFiveNote.1")
						.deadlineWith(new ReadyShooter(Constants.Shooter.readyIntake, false)),
					new IntakeGround().withTimeout(2),
					Autonomous
						.dynamic("MiddleFiveNote.3")
						.deadlineWith(new ReadyShooter(Constants.Shooter.readyShootRear, true)),
					new ShootSpeaker(false, 2),
					new ReadyShooter(Constants.Shooter.readyIntake, false),
					new IntakeGround().withTimeout(2),
					Autonomous
						.dynamic("MiddleFiveNote.4")
						.deadlineWith(new ReadyShooter(Constants.Shooter.readyShootRear, true)),
					new ShootSpeaker(false, 2),
					new ReadyShooter(Constants.Shooter.readyIntake, false),
					new IntakeGround().withTimeout(2),
					Autonomous
						.dynamic("MiddleFiveNote.5")
						.deadlineWith(new ReadyShooter(Constants.Shooter.readyShootRear, true)),
					new ShootSpeaker(false, 2),
					Autonomous.path("MiddleFiveNote.5"),
					new IntakeGround().withTimeout(2),
					Autonomous
						.dynamicThen("MiddleFiveNote.6")
						.deadlineWith(
							new WaitCommand(2).andThen(new ReadyShooter(Constants.Shooter.readyShootRear, true))
						),
					new ShootSpeaker(false, 2)
				)
			);

		chooser
			.addOption(
				"[testing] Source Side Center Note",
				new SequentialCommandGroup(
					Autonomous.setInitialPose("SourceSideCenterNote.1"),
					new ReadyShooter(Constants.Shooter.readyShootRear, true),
					new ShootSpeaker(false, 2),
					Autonomous.path("SourceSideCenterNote.1"),
					new IntakeGround().withTimeout(2),
					Autonomous.dynamic("SourceSideCenterNote.2"),
					new ParallelCommandGroup(
						new ReadyShooter(Constants.Shooter.readyShootRear, true),
						Autonomous.path("SourceSideCenterNote.2")
					),
					new ShootSpeaker(false, 2)
				)
			);

		chooser
			.addOption(
				"[comp] Drive",
				new SequentialCommandGroup(new ReadyShooter(Constants.Shooter.readyShootRear, true), new DriveTime(1))
			);

		chooser
			.addOption(
				"[comp] Shoot/Drive",
				new SequentialCommandGroup(
					new ReadyShooter(Constants.Shooter.readyShootRear, true),
					new ShootSpeaker(false, 2),
					new DriveTime(1)
				)
			);

		chooser
			.addOption(
				"[comp] Shoot",
				new SequentialCommandGroup(
					new ReadyShooter(Constants.Shooter.readyShootRear, true),
					new ShootSpeaker(false, 2)
				)
			);

		chooser
			.addOption(
				"[testing] Amp Side Center Note",
				new SequentialCommandGroup(
					Autonomous.setInitialPose("AmpSideCenterNote.1"),
					new ShootSpeaker(false, 2),
					Autonomous.path("AmpSideCenterNote.1"),
					new IntakeGround().withTimeout(2),
					Autonomous
						.dynamicThen("AmpSideCenterNote.2")
						.alongWith(new ReadyShooter(Constants.Shooter.readyShootRear, true)),
					new ShootSpeaker(false, 2),
					Autonomous.path("AmpSideCenterNote.3"),
					new IntakeGround().withTimeout(2),
					Autonomous
						.dynamicThen("AmpSideCenterNote.4")
						.alongWith(new ReadyShooter(Constants.Shooter.readyShootRear, true)),
					new ShootSpeaker(false, 2)
				)
			);

		chooser
			.addOption(
				"[testing] Delayed Amp Side Center Note",
				new SequentialCommandGroup(
					Autonomous.setInitialPose("AmpSideCenterNote.1"),
					new WaitCommand(10),
					new ShootSpeaker(false, 2),
					Autonomous.path("AmpSideCenterNote.1"),
					new IntakeGround().withTimeout(2),
					Autonomous
						.dynamicThen("AmpSideCenterNote.2")
						.alongWith(new ReadyShooter(Constants.Shooter.readyShootRear, true)),
					new ShootSpeaker(false, 2),
					Autonomous.path("AmpSideCenterNote.3"),
					new IntakeGround().withTimeout(2),
					Autonomous
						.dynamicThen("AmpSideCenterNote.4")
						.alongWith(new ReadyShooter(Constants.Shooter.readyShootRear, true)),
					new ShootSpeaker(false, 2)
				)
			);

		chooser
			.addOption(
				"[comp] Two Note",
				new SequentialCommandGroup(
					new ShootSpeaker(false, 2),
					new IntakeGround().withTimeout(2),
					new ShootSpeaker(false, 2)
				)
			);

		chooser
			.addOption(
				"[testing] Source Side Centerline Disruption",
				new SequentialCommandGroup(
					Autonomous.setInitialPose("SourceSideCenterDisrupt.1"),
					new ReadyShooter(Constants.Shooter.readyShootRear, true),
					new ShootSpeaker(false, 2),
					Autonomous.path("SourceSideCenterDisrupt.1"),
					new IntakeGround().withTimeout(2)
				)
			);

		return chooser;
	}

	public static Command setInitialPose(final String name) {
		final PathPlannerPath traj = PathPlannerPath.fromChoreoTrajectory(name);
		final Pose2d initial = traj.getPathPoses().get(0);

		return Commands.runOnce(() -> {
			Robot.cont.drivetrain
				.reset(
					new Pose2d(
						Autonomous.getPoseForAlliance(initial).getTranslation(),
						Robot.cont.drivetrain.est.getEstimatedPosition().getRotation()
					)
				);

			Logger.recordOutput("Drivetrain/Auto/x0", initial.getX());
			Logger.recordOutput("Drivetrain/Auto/y0", initial.getY());
			Logger.recordOutput("Drivetrain/Auto/r0", initial.getRotation().getDegrees());
		});
	}

	public static Command path(final String name) {
		final PathPlannerPath choreoPath = PathPlannerPath.fromChoreoTrajectory(name);
		return AutoBuilder.followPath(choreoPath);
	}

	public static Command dynamic(final String next) {
		final ChoreoTrajectory traj = Choreo.getTrajectory(next);

		return AutoBuilder
			.pathfindToPoseFlipped(
				traj.getInitialPose(),
				new PathConstraints(
					Constants.Drivetrain.maxVelocity.in(Units.MetersPerSecond),
					2,
					Constants.Drivetrain.maxAngularVelocity.in(Units.RadiansPerSecond),
					2
				)
			)
			.alongWith(
				new InstantCommand(() -> Logger.recordOutput("Drivetrain/Auto/DynamicTarget", traj.getInitialPose()))
			);
	}

	public static Command dynamicThen(final String next) {
		final PathPlannerPath traj = PathPlannerPath.fromChoreoTrajectory(next);

		return AutoBuilder
			.pathfindThenFollowPath(
				traj,
				new PathConstraints(
					Constants.Drivetrain.maxVelocity.in(Units.MetersPerSecond),
					3,
					Constants.Drivetrain.maxAngularVelocity.in(Units.RadiansPerSecond),
					2
				)
			)
			.alongWith(
				new InstantCommand(
					() -> Logger
						.recordOutput("Drivetrain/Auto/DynamicTarget", Choreo.getTrajectory(next).getInitialPose())
				)
			);
	}

	/*
	 * Returns the original or mirrored pose depending on alliance color (since the field is flipped)
	 */
	private static Pose2d getPoseForAlliance(final Pose2d initialPose) {
		if(DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
			return new Pose2d(
				initialPose.getX(),
				Constants.fieldDepth.in(Units.Meters) - initialPose.getY(),
				initialPose.getRotation().unaryMinus()
			);
		} else return initialPose;
	}
}
