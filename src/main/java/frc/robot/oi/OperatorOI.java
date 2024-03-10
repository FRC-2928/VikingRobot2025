package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.climber.Initialize;
import frc.robot.subsystems.ShooterIO;

public class OperatorOI extends BaseOI {
	public OperatorOI(final CommandXboxController controller) {
		super(controller);

		this.climberDown = this.controller.a();
		this.climberUp = this.controller.y();
		this.climberUpSlow = this.controller.x();

		this.climberOverrideLower = this.controller.povDown();
		this.climberOverrideRaise = this.controller.povUp();

		this.initializeClimber = this.controller.rightStick();

		this.reject = this.controller.b();

		this.overrideShoot = this.controller.rightTrigger();
	}

	public final Trigger climberDown;
	public final Trigger climberUp;
	public final Trigger climberUpSlow;

	public final Trigger climberOverrideLower;
	public final Trigger climberOverrideRaise;

	public final Trigger initializeClimber;

	public final Trigger reject;

	public final Trigger overrideShoot;

	public void configureControls() {
		this.climberDown.whileTrue(new RunCommand(() -> Robot.cont.climber.io.set(0)));
		this.climberUp.whileTrue(new RunCommand(() -> {
			Robot.cont.climber.io.fast(true);
			Robot.cont.climber.io.set(Constants.Climber.max);
		}));
		this.climberUpSlow.whileTrue(new RunCommand(() -> {
			Robot.cont.climber.io.fast(false);
			Robot.cont.climber.io.set(Constants.Climber.max);
		}));

		this.climberOverrideLower.whileTrue(new FunctionalCommand(() -> {
		}, () -> Robot.cont.climber.io.override(-1), a -> Robot.cont.climber.io.override(0), () -> false));
		this.climberOverrideRaise.whileTrue(new FunctionalCommand(() -> {
		}, () -> {
			Robot.cont.climber.io.fast(true);
			Robot.cont.climber.io.override(1);
		}, interrupted -> Robot.cont.climber.io.override(0), () -> false));

		this.initializeClimber.onTrue(new Initialize());

		this.reject.whileTrue(new FunctionalCommand(() -> {
		},
			() -> Robot.cont.shooter.io.runIntake(ShooterIO.Demand.Reverse),
			interrupt -> Robot.cont.shooter.io.runIntake(ShooterIO.Demand.Halt),
			() -> false,
			Robot.cont.shooter
		));
	}
}
