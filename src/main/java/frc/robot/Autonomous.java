package frc.robot;

import java.util.Map;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CoralPosition;
import frc.robot.Constants.GamePieceType;
import frc.robot.Constants.ReefPosition;
import frc.robot.commands.drivetrain.VoltageRampCommand;

public final class Autonomous {
	public static final Pose2d autoPosLeftBlue = new Pose2d(7.37,5.68,new Rotation2d(4.19));
	public static final Pose2d autoPosCenterBlue = new Pose2d(7.27,4.15,new Rotation2d(3.13));
	public static final Pose2d autoPosRightBlue = new Pose2d(7.07,2.44,new Rotation2d(2.09));
	public static final String[] AutoRoutines = {"[Comp] SimpleFromRight"};
	public static final Map<String,Pose2d> autoMapBlue = Map.of(
		"[Comp] Score2CoralFromLeft", autoPosLeftBlue, 
		"[Comp] Score1CoralFromCenter", autoPosCenterBlue,
		"[Comp] Score2CoralFromRight", autoPosRightBlue
	);

	public static Pose2d getAutoStartingPose(final String routine, final Alliance currentAlliance) {
		final Pose2d bluePose = autoMapBlue.getOrDefault(routine, new Pose2d(-10, -10, Rotation2d.kZero));
		if (currentAlliance == Alliance.Blue) {
			return bluePose;
		}
		return bluePose.rotateAround(new Translation2d(Constants.FIELD_LAYOUT.getFieldLength()/2, Constants.FIELD_LAYOUT.getFieldWidth()/2), Rotation2d.k180deg);
	}

	public static AutoChooser getChoreoAutoChooser() {
		final AutoChooser choreoChooser = new AutoChooser();
		AutoFactory autoFactory = RobotContainer.getInstance().drivetrain.autoFactory;

		choreoChooser.addCmd("[Comp] Score2CoralFromRight", () -> Commands.sequence(
			new InstantCommand(() -> {RobotContainer.getInstance().elevator.setTargetCoralLevel(CoralPosition.L1);}),	
			RobotContainer.getInstance().elevator.goToGamePieceHeight(GamePieceType.CORAL).withTimeout(0.2),
			new InstantCommand(() -> {RobotContainer.getInstance().elevator.onEjectCoral();
										RobotContainer.getInstance().elevator.setTargetCoralLevel(CoralPosition.NONE);}),
			new InstantCommand(() -> {RobotContainer.getInstance().elevator.setTargetCoralLevel(CoralPosition.L4);}, RobotContainer.getInstance().elevator),
			autoFactory.trajectoryCmd("StartRightToE"),
			RobotContainer.getInstance().autoScoreCoral(ReefPosition.E),
			autoFactory.trajectoryCmd("EToB1Reverse"),
			new ParallelDeadlineGroup(
				new SequentialCommandGroup(
					RobotContainer.getInstance().bananaFlywheels.intakeUntilLimits(),
					RobotContainer.getInstance().bananaFlywheels.advanceCoralTimeBased()
				),
				RobotContainer.getInstance().intake.runTrough()/*,
				new RepeatCommand(CenterLimelight.centerLimelightHPReverse(HumanPlayerPosition.B1)*/
			).withTimeout(3),
			autoFactory.trajectoryCmd("B1ReverseToD"),
			new ConditionalCommand(
				new InstantCommand(),
				new ParallelDeadlineGroup(
					new SequentialCommandGroup(
						RobotContainer.getInstance().bananaFlywheels.intakeUntilLimits(),
						RobotContainer.getInstance().bananaFlywheels.advanceCoralTimeBased()
					),
					RobotContainer.getInstance().intake.runTrough()
				),
				() -> RobotContainer.getInstance().bananaFlywheels.holdingCoral()
			),
			RobotContainer.getInstance().autoScoreCoral(ReefPosition.D)
		));
		

		choreoChooser.addCmd("[Comp] Drive forward", () -> new RunCommand(() -> {
			RobotContainer.getInstance().drivetrain.control(new ChassisSpeeds(3,0,0));
			}).withTimeout(0.5)
		);
		

		choreoChooser.addCmd("[Comp] Score2CoralFromLeft", () -> Commands.sequence(
			new InstantCommand(() -> {RobotContainer.getInstance().elevator.setTargetCoralLevel(CoralPosition.L1);}),	
			RobotContainer.getInstance().elevator.goToGamePieceHeight(GamePieceType.CORAL).withTimeout(0.2),
			new InstantCommand(() -> {RobotContainer.getInstance().elevator.onEjectCoral();
										RobotContainer.getInstance().elevator.setTargetCoralLevel(CoralPosition.NONE);}),
			new InstantCommand(() -> {RobotContainer.getInstance().elevator.setTargetCoralLevel(CoralPosition.L4);}, RobotContainer.getInstance().elevator),
			autoFactory.trajectoryCmd("StartLeftToJ"),
			RobotContainer.getInstance().autoScoreCoral(ReefPosition.J),
			autoFactory.trajectoryCmd("JToA2Reverse").andThen(RobotContainer.getInstance().drivetrain.haltCommand()),
			new ParallelDeadlineGroup(
				new SequentialCommandGroup(
					RobotContainer.getInstance().bananaFlywheels.intakeUntilLimits(),
					RobotContainer.getInstance().bananaFlywheels.advanceCoralTimeBased()
				),
				RobotContainer.getInstance().intake.runTrough()/*,
				new RepeatCommand(CenterLimelight.centerLimelightHPReverse(HumanPlayerPosition.A2)*/
			).withTimeout(3),
			autoFactory.trajectoryCmd("A2ReverseToK").andThen(RobotContainer.getInstance().drivetrain.haltCommand()),
			new ConditionalCommand(
				new InstantCommand(),
				new ParallelDeadlineGroup(
					new SequentialCommandGroup(
						RobotContainer.getInstance().bananaFlywheels.intakeUntilLimits(),
						RobotContainer.getInstance().bananaFlywheels.advanceCoralTimeBased()
					),
					RobotContainer.getInstance().intake.runTrough()
				),
				() -> RobotContainer.getInstance().bananaFlywheels.holdingCoral()
			),
			RobotContainer.getInstance().autoScoreCoral(ReefPosition.K)
		));

		choreoChooser.addCmd("[Comp] Score1CoralFromCenter", () -> Commands.sequence(
			new InstantCommand(() -> {RobotContainer.getInstance().elevator.setTargetCoralLevel(CoralPosition.L1);}),	
			RobotContainer.getInstance().elevator.goToGamePieceHeight(GamePieceType.CORAL).withTimeout(0.2),
			new InstantCommand(() -> {RobotContainer.getInstance().elevator.onEjectCoral();
										RobotContainer.getInstance().elevator.setTargetCoralLevel(CoralPosition.NONE);}),
			new InstantCommand(() -> {RobotContainer.getInstance().elevator.setTargetCoralLevel(CoralPosition.L4);}, RobotContainer.getInstance().elevator),
			// autoFactory.trajectoryCmd("SimpleScore"),
			RobotContainer.getInstance().autoScoreCoral(ReefPosition.H),
			autoFactory.trajectoryCmd("HToBackOff"),
			RobotContainer.getInstance().drivetrain.haltCommand()
		));

		choreoChooser.addCmd(
				"[Test] Forward Back",
				() -> new SequentialCommandGroup(
					autoFactory.resetOdometry("forwardBack"),
					autoFactory.trajectoryCmd("forwardBack")
				)
			);
		
		choreoChooser.addCmd("[Test] Voltage ramp", () -> new VoltageRampCommand());

		return choreoChooser;
	}
}
