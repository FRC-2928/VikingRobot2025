package frc.robot.commands.shooter;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.ShooterIO.Demand;

public class IntakeSource extends Command {
	public IntakeSource() { this.addRequirements(Robot.cont.shooter); }

	@Override
	public void execute() {
		Robot.cont.shooter.io.rotate(Constants.Shooter.intakeSource);
		Robot.cont.shooter.io.runFlywheels(Units.RotationsPerSecond.of(-60));
		Robot.cont.shooter.io.runFeeder(Demand.Reverse);
	}

	@Override
	public void end(final boolean interrupted) {
		Robot.cont.shooter.io
			.rotate(
				Robot.cont.shooter.inputs.holdingNote ? Constants.Shooter.readyDrive : Constants.Shooter.readyShootRear
			);
		Robot.cont.shooter.io.runFlywheels(Units.RotationsPerSecond.zero());
		Robot.cont.shooter.io.runFeeder(Demand.Halt);
	}

	@Override
	public boolean isFinished() { return Robot.cont.shooter.inputs.holdingNote; }
}
