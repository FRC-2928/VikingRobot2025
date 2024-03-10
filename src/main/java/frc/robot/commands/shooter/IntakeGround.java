package frc.robot.commands.shooter;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.ShooterIO.Demand;

public class IntakeGround extends Command {
	public static double lastTime = 0; // this is a bad way to do this but its necessary for right now, please do real path planning in the future

	public IntakeGround(final boolean correction) {
		this.correction = correction;

		this.addRequirements(Robot.cont.shooter);
		if(correction) this.addRequirements(Robot.cont.drivetrain);
	}

	private final boolean correction;
	private double start;

	@Override
	public void initialize() { this.start = Timer.getFPGATimestamp(); }

	@Override
	public void execute() {
		final boolean pivotReady = Math
			.abs(
				Robot.cont.shooter.inputs.angle.in(Units.Degrees) - Constants.Shooter.intakeGround.in(Units.Degrees)
			) <= 1.5;

		Robot.cont.shooter.io.rotate(Constants.Shooter.intakeGround);
		Robot.cont.shooter.io.runFlywheels(-0.25);
		Robot.cont.shooter.io.runFeeder(Demand.Reverse);
		Robot.cont.shooter.io.runIntake(pivotReady ? Demand.Reverse : Demand.Halt);

		if(this.correction) {
			Robot.cont.drivetrain
				.control(
					Robot.cont.drivetrain.joystickSpeeds
						.plus(
							Robot.cont.drivetrain
								.rod(
									new ChassisSpeeds(
										-0.5,
										Robot.cont.drivetrain.limelightNote
											.getTargetHorizontalOffset()
											.in(Units.Rotations)
											* 10,
										0
									).times(pivotReady ? 1 : 1)
								)
						)
				);
		}
	}

	@Override
	public void end(final boolean interrupted) {
		Robot.cont.shooter.io
			.rotate(
				Robot.cont.shooter.inputs.holdingNote ? Constants.Shooter.readyDrive : Constants.Shooter.readyIntake
			);
		Robot.cont.shooter.io.runFlywheels(0);
		Robot.cont.shooter.io.runFeeder(Demand.Halt);
		Robot.cont.shooter.io.runIntake(Demand.Halt);

		IntakeGround.lastTime = Timer.getFPGATimestamp() - this.start;
	}

	@Override
	public boolean isFinished() { return Robot.cont.shooter.inputs.holdingNote; }
}
