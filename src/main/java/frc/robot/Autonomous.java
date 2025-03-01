package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ReefPosition;
import frc.robot.commands.drivetrain.CenterLimelight;
import frc.robot.commands.drivetrain.VoltageRampCommand;

public final class Autonomous {

	public static AutoChooser getChoreoAutoChooser() {
		final AutoChooser choreoChooser = new AutoChooser();
		AutoFactory autoFactory = Robot.cont.drivetrain.autoFactory;

		choreoChooser.addCmd("[Comp] SimpleFromRight", () -> Commands.sequence(
			autoFactory.trajectoryCmd("StartToF"),
			Commands.deadline(new WaitCommand(2), CenterLimelight.centerLimeLightPosition(ReefPosition.F)),
			autoFactory.trajectoryCmd("FToB2Reverse"),
			Commands.deadline(new WaitCommand(2), CenterLimelight.centerLimelightB2Reverse()),
			autoFactory.trajectoryCmd("B1ReverseToC"),
			Commands.deadline(new WaitCommand(2), CenterLimelight.centerLimeLightPosition(ReefPosition.C)),
			autoFactory.trajectoryCmd("CToB1Reverse"),
			Commands.deadline(new WaitCommand(2), CenterLimelight.centerLimelightB2Reverse()),
			autoFactory.trajectoryCmd("B1ReverseToD"),
			Commands.deadline(new WaitCommand(2), CenterLimelight.centerLimeLightPosition(ReefPosition.D))
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
