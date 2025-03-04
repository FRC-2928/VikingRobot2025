package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.climber.Initialize;
import frc.robot.commands.drivetrain.JoystickDrive;

public class OperatorOI extends BaseOI {
	public OperatorOI(final CommandXboxController controller) {
		super(controller);

		this.climberDown = this.controller.x();
		this.climberUp = this.controller.y();

		this.climberOverrideLower = this.controller.povDown();
		this.climberOverrideRaise = this.controller.povUp();

		this.initializeClimber = this.controller.rightStick();
		
		this.intakeIn = this.controller.a();
		this.intakeOut = this.controller.b();
		this.slowdTrigger = this.controller.x();
		this.hault = this.controller.y();
		this.foc = this.controller.rightBumper();
		this.resetAngle = this.controller.back();

		// this.fixedShoot = this.controller.leftTrigger();
		// this.overrideShoot = this.controller.rightTrigger();
	}

	public final Trigger climberDown;
	public final Trigger climberUp;
	public final Trigger foc;
	public final Trigger climberOverrideLower;
	public final Trigger climberOverrideRaise;
	public final Trigger resetAngle;
	public final Trigger initializeClimber;
	// public final Trigger raiseElevator;
	// public final Trigger lowerElivator;
	public final Trigger intakeOut;
	public final Trigger intakeIn;
	// public final Trigger fixedShoot;

	// public final Trigger overrideShoot;
	public final Trigger hault;
	public final Trigger slowdTrigger;
	public void configureControls() {
		this.resetAngle.whileTrue(new RunCommand(Robot.cont.drivetrain::seedLimelightImu)).whileFalse(new RunCommand(Robot.cont.drivetrain::setImuMode2));
		this.climberDown.whileTrue(new RunCommand(() -> Robot.cont.climber.io.set(0)));
		this.climberUp.whileTrue(new RunCommand(() -> Robot.cont.climber.io.set(Constants.Climber.max)));
		this.slowdTrigger.whileFalse(new RunCommand(() -> JoystickDrive.setSlowMode(false)))
		.whileTrue(new RunCommand(() -> JoystickDrive.setSlowMode(true)));
		this.hault.whileTrue(new RunCommand(() -> Robot.cont.drivetrain.halt()));
		this.climberOverrideLower.whileTrue(new FunctionalCommand(() -> {
		}, () -> Robot.cont.climber.io.override(-1), interrupted -> Robot.cont.climber.io.override(0), () -> false));
		this.climberOverrideRaise.whileTrue(new FunctionalCommand(() -> {
		}, () -> Robot.cont.climber.io.override(1), interrupted -> Robot.cont.climber.io.override(0), () -> false));
		this.initializeClimber.onTrue(new Initialize());
	}
}
//change elevator height
//create dashboard w/ shuffle board