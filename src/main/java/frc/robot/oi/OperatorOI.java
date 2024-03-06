package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.climber.Initialize;

public class OperatorOI extends BaseOI {
	public OperatorOI(final CommandXboxController controller) {
		super(controller);

		this.lowerClimber = this.controller.a();
		this.raiseClimber = this.controller.y();
		this.raiseClimberSlow = this.controller.b();

		this.initializeClimber = this.controller.rightStick();
	}

	public final Trigger lowerClimber;
	public final Trigger raiseClimber;
	public final Trigger raiseClimberSlow;

	public final Trigger initializeClimber;

	public void configureControls() {
		this.lowerClimber.whileTrue(new RunCommand(() -> Robot.cont.climber.io.set(0, true)));
		this.raiseClimber.whileTrue(new RunCommand(() -> Robot.cont.climber.io.set(Constants.Climber.max, true)));
		this.raiseClimberSlow.whileTrue(new RunCommand(() -> Robot.cont.climber.io.set(Constants.Climber.max, false)));

		this.initializeClimber.onTrue(new Initialize());
	}
}
