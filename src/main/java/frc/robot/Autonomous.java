package frc.robot;

import java.util.Map;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.CoralPosition;
import frc.robot.Constants.ReefPosition;
import frc.robot.commands.drivetrain.CenterLimelight;
import frc.robot.commands.drivetrain.VoltageRampCommand;

public final class Autonomous {
	public static final Pose2d autoPosLeft = new Pose2d(0.0,0.0,new Rotation2d());
	public static final Pose2d autoPosCenter = new Pose2d(0.0,0.0,new Rotation2d());
	public static final Pose2d autoPosRight = new Pose2d(0.0,0.0,new Rotation2d());
	public static final String[] AutoRoutines = {"[Comp] SimpleFromRight"};
	public static final Map<String,Pose2d> autoMap = Map.of(
		"initial", autoPosLeft, 
		"2", autoPosCenter,
		AutoRoutines[0], autoPosRight
	);
	public static AutoChooser getChoreoAutoChooser() {
		final AutoChooser choreoChooser = new AutoChooser();
		AutoFactory autoFactory = Robot.cont.drivetrain.autoFactory;

		choreoChooser.addCmd("[Comp] Score2CoralFromRight", () -> Commands.sequence(
			new InstantCommand(() -> {Robot.cont.elevator.setTargetCoralLevel(CoralPosition.L4);}, Robot.cont.elevator),
			autoFactory.trajectoryCmd("StartRightToE"),
			Robot.cont.autoScoreCoral(ReefPosition.E),
			autoFactory.trajectoryCmd("EToB2Reverse"),
			Commands.deadline(new WaitCommand(2), CenterLimelight.centerLimelightB2Reverse()),
			Robot.cont.passCoral(),
			autoFactory.trajectoryCmd("B2ReverseToD"),
			Robot.cont.autoScoreCoral(ReefPosition.D)
		));

		choreoChooser.addCmd("[Comp] SimpleScore", () -> Commands.sequence(autoFactory.trajectoryCmd("SimpleScore")));

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
