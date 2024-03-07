package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class Initialize extends Command {
	public Initialize() {
		// todo: run up an inch, then down until we hit a limit switch or stall
		this.addRequirements(Robot.cont.climber);
	}

	private boolean descending;
	private double start;

	@Override
	public void initialize() {
		this.descending = false;
		this.start = Robot.cont.climber.inputs.position;
	}

	@Override
	public void execute() {
		if(this.descending) {
			Robot.cont.climber.io.set(this.start - 1000, false);
		} else {
			Robot.cont.climber.io.set(this.start + 2, false);
			if(Robot.cont.climber.inputs.position > this.start + Constants.Climber.initializeRaiseDistance)
				this.descending = true;
		}
	}

	@Override
	public void end(final boolean interrupted) { Robot.cont.climber.io.offset(0, false); }

	@Override
	public boolean isFinished() { return Robot.cont.climber.inputs.home; }
}
