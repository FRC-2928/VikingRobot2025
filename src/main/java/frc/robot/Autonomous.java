package frc.robot;

import java.util.Map;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.CoralPosition;
import frc.robot.Constants.HumanPlayerPosition;
import frc.robot.Constants.ReefPosition;
import frc.robot.commands.drivetrain.CenterLimelight;
import frc.robot.commands.drivetrain.VoltageRampCommand;

public final class Autonomous {
	public static final Pose2d autoPosLeft = new Pose2d(0.0,0.0,new Rotation2d());
	public static final Pose2d autoPosCenter = new Pose2d(0.0,0.0,new Rotation2d());
	public static final Pose2d autoPosRight = new Pose2d(0.0,0.0,new Rotation2d());
	public static final String[] AutoRoutines = {"[Comp] SimpleFromRight"};
	public static final Map<String,Pose2d> autoMap = Map.of(
		"[Comp] Score2CoralFromLeft", autoPosLeft, 
		"[Comp] Score1CoralFromCenter", autoPosCenter,
		"[Comp] Score2CoralFromRight", autoPosRight
	);
	public static AutoChooser getChoreoAutoChooser() {
		final AutoChooser choreoChooser = new AutoChooser();
		AutoFactory autoFactory = RobotContainer.getInstance().drivetrain.autoFactory;

		choreoChooser.addCmd("[Comp] Score2CoralFromRight", () -> Commands.sequence(
			new InstantCommand(() -> {RobotContainer.getInstance().elevator.setTargetCoralLevel(CoralPosition.L4);}, RobotContainer.getInstance().elevator),
			autoFactory.trajectoryCmd("StartRightToE"),
			RobotContainer.getInstance().autoScoreCoral(ReefPosition.E),
			autoFactory.trajectoryCmd("EToB1Reverse"),
			Commands.deadline(new WaitCommand(2), CenterLimelight.centerLimelightHPReverse(HumanPlayerPosition.B1)),
			RobotContainer.getInstance().passCoral(),
			autoFactory.trajectoryCmd("B1ReverseToD"),
			RobotContainer.getInstance().autoScoreCoral(ReefPosition.D)
		));

		choreoChooser.addCmd("[Comp] Score2CoralFromLeft", () -> Commands.sequence(
			new InstantCommand(() -> {RobotContainer.getInstance().elevator.setTargetCoralLevel(CoralPosition.L4);}, RobotContainer.getInstance().elevator),
			autoFactory.trajectoryCmd("StartLeftToJ"),
			RobotContainer.getInstance().autoScoreCoral(ReefPosition.L),
			autoFactory.trajectoryCmd("JToA2Reverse"),
			Commands.deadline(new WaitCommand(2), CenterLimelight.centerLimelightHPReverse(HumanPlayerPosition.A2),
			RobotContainer.getInstance().passCoral(),
			autoFactory.trajectoryCmd("A2ReverseToK"),
			RobotContainer.getInstance().autoScoreCoral(ReefPosition.K))
		));

		choreoChooser.addCmd("[Comp] Score1CoralFromCenter", () -> Commands.sequence(
			new InstantCommand(() -> {RobotContainer.getInstance().elevator.setTargetCoralLevel(CoralPosition.L4);}, RobotContainer.getInstance().elevator),
			autoFactory.trajectoryCmd("SimpleScore"),
			RobotContainer.getInstance().autoScoreCoral(ReefPosition.H),
			autoFactory.trajectoryCmd("HToBackOff")
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
