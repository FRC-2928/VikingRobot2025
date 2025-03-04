package frc.robot.oi;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.commands.drivetrain.JoystickDrive;

public class OperatorOI extends BaseOI {
	public OperatorOI(final CommandXboxController controller) {
		super(controller);

		this.climberDown = this.controller.x();
		this.climberUp = this.controller.y();
		this.climbMotion = this.controller::getLeftY;

		this.climberOverrideLower = this.controller.povDown();
		this.climberOverrideRaise = this.controller.povUp();

		this.initializeClimber = this.controller.rightStick();
		
		this.intakeIn = this.controller.a();
		this.intakeOut = this.controller.b();
		this.slowdTrigger = this.controller.x();
		this.hault = this.controller.y();
		this.foc = this.controller.rightBumper();
		this.resetAngle = this.controller.back();
		this.toggleClimb = this.controller.leftBumper();
		this.toggleReefHeightDown = this.controller.povDown();
		this.toggleReefHeightUp = this.controller.povUp();
		this.passOffCoral = this.controller.rightTrigger();
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
	public final Trigger toggleClimb;
	// public final Trigger raiseElevator;
	// public final Trigger lowerElivator;
	public final Trigger intakeOut;
	public final Trigger intakeIn;
	public final Supplier<Double> climbMotion;
	public final Trigger toggleReefHeightUp;
	public final Trigger toggleReefHeightDown;
	public final Trigger passOffCoral;
	// public final Trigger fixedShoot;

	// public final Trigger overrideShoot;
	public final Trigger hault;
	public final Trigger slowdTrigger;
	public void configureControls() {
		this.slowdTrigger.whileTrue(new JoystickDrive(Robot.cont.drivetrain, 0.3));
		this.hault.whileTrue(new RunCommand(() -> Robot.cont.drivetrain.halt()));
		toggleClimb.onTrue(Robot.cont.elevator.toggleClimbMode());
		this.toggleReefHeightDown.onTrue(new InstantCommand(Robot.cont.elevator::toggleReefHeightDown));
		this.toggleReefHeightUp.onTrue(new InstantCommand(Robot.cont.elevator::toggleReefHeightUp));
		this.passOffCoral.whileTrue(Robot.cont.passCoral());
	}
}
//change elevator height
//create dashboard w/ shuffle board