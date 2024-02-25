package frc.robot.commands.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainModifier;

public class TempTest extends DrivetrainModifier.Modification {
	@Override
	public ChassisSpeeds modify(final ChassisSpeeds control) {
		return new ChassisSpeeds(
			control.vxMetersPerSecond,
			control.vyMetersPerSecond + 0.25,
			control.omegaRadiansPerSecond
		);
	}

	@Override
	public void end(final boolean interrupted) { Robot.cont.diag.chirp(700, 500); }
}
