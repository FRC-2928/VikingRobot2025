package frc.robot.commands.climber;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class Initialize extends Command {
	public Initialize() {
		// todo: run up an inch, then down until we hit a limit switch or stall
		this.addRequirements(Robot.cont.climber);
	}

	@Override
	public void initialize() {
		Robot.cont.climber.io.drive(Units.InchesPerSecond.of(-2));
		if(Robot.cont.climber.inputs.limit) Robot.cont.climber.io.zero();
	}

	@Override
	public boolean isFinished() { return Robot.cont.climber.inputs.limit; }
}
