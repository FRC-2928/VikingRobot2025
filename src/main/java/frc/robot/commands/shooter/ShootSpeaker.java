package frc.robot.commands.shooter;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.ShooterIO.Demand;

public class ShootSpeaker extends Command {
	public ShootSpeaker() { this.addRequirements(Robot.cont.shooter); }

	private boolean fired;

	@Override
	public void initialize() { this.fired = false; }

	@Override
	public void execute() {
		Robot.cont.shooter.io.rotate(Constants.Shooter.readyShootFront);
		Robot.cont.shooter.io.runFlywheels(Units.RotationsPerSecond.of(90));
		if(
			Robot.cont.shooter.inputs.flywheelSpeed
				.in(Units.RotationsPerSecond) >= Constants.Shooter.flywheelSpeedThreshold.in(Units.RotationsPerSecond)
				|| this.fired
		) {
			Robot.cont.shooter.io.runFeeder(Demand.Forward);
			this.fired = true;
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
	}
}

// TODO: implement aiming
// fwd/back based on pose
