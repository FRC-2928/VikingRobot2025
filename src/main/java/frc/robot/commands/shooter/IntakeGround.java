package frc.robot.commands.shooter;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.ShooterIO.Demand;

public class IntakeGround extends Command {
	public IntakeGround(final boolean correction) {
		this.correction = correction;

		this.addRequirements(Robot.cont.shooter);
		if(correction) this.addRequirements(Robot.cont.drivetrain);
	}

	private final boolean correction;

	@Override
	public void execute() {
		Robot.cont.shooter.io.rotate(Constants.Shooter.intake);
		Robot.cont.shooter.io.runFlywheels(Units.RotationsPerSecond.of(-60));
		Robot.cont.shooter.io.runFeeder(Demand.Reverse);
		Robot.cont.shooter.io
			.runIntake(
				Math
					.abs(
						Robot.cont.shooter.inputs.angle.in(Units.Degrees) - Constants.Shooter.intake.in(Units.Degrees)
					) <= 3 ? Demand.Reverse : Demand.Halt
			);

		if(this.correction) {
			Robot.cont.drivetrain
				.control(
					Robot.cont.drivetrain.joystickSpeeds
						.plus(
							Robot.cont.drivetrain
								.rod(
									new ChassisSpeeds(
										Robot.cont.drivetrain.limelightNote
											.getTargetHorizontalOffset()
											.in(Units.Rotations),
										1,
										0
									)
								)
								.times(0)
						)
				);
		}
	}

	@Override
	public void end(final boolean interrupted) {
		Robot.cont.shooter.io
			.rotate(
				Robot.cont.shooter.inputs.holdingNote
					? Constants.Shooter.readyShootFront
					: Constants.Shooter.readyIntake
			);
		Robot.cont.shooter.io.runFlywheels(Units.RotationsPerSecond.zero());
		Robot.cont.shooter.io.runFeeder(Demand.Halt);
		Robot.cont.shooter.io.runIntake(Demand.Halt);
	}

	@Override
	public boolean isFinished() { return Robot.cont.shooter.inputs.holdingNote; }
}
