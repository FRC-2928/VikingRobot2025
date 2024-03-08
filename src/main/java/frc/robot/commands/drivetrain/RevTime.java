package frc.robot.commands.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;

public class RevTime extends Command {
	public RevTime(final double time) {
		this.addRequirements(Robot.cont.drivetrain);

		this.time = time;
	}

	public final double time;

	private double end;

	@Override
	public void initialize() { this.end = Timer.getFPGATimestamp() + this.time; }

	@Override
	public void execute() { Robot.cont.drivetrain.control(Robot.cont.drivetrain.rod(new ChassisSpeeds(2, 0, 0))); }

	@Override
	public boolean isFinished() { return Timer.getFPGATimestamp() >= this.end; }
}
