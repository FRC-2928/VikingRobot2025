package frc.robot.oi;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.GamePieceType;
import frc.robot.RobotContainer;

public class OperatorOI extends BaseOI {
	public OperatorOI(final CommandXboxController controller) {
		super(controller);

		this.climbMotion = this.controller::getLeftY;

		// this.climberOverrideLower = this.controller.povDown();
		// this.climberOverrideRaise = this.controller.povUp();

		this.initializeClimber = this.controller.rightStick();
		
		// this.intakeIn = this.controller.a();
		// this.intakeOut = this.controller.b();
		// this.slowedTrigger = this.controller.x();
		// this.halt = this.controller.y();
		this.setElevatorModeCoral = this.controller.rightBumper();
		// this.resetAngle = this.controller.back();
		// this.haultElevatpr = this.controller.leftBumper();
		this.setElevatorModeAlgae = this.controller.leftBumper();
		this.setElevatorModeNone = this.controller.back();
		this.climbModeOn = new Trigger(() -> (RobotContainer.getInstance().elevator.hasCurrentGamePieceType(GamePieceType.CAGE)));
		this.homeElevator = this.controller.rightTrigger();
		
		this.toggleReefHeightDown = this.controller.povDown();
		this.toggleReefHeightUp = this.controller.povUp();
		this.passOffCoral = this.controller.leftTrigger();

		this.alignElevatorCoral = new Trigger(() -> (this.controller.b().getAsBoolean())/* ) .and(RobotContainer.getInstance().driverOI.holdingCoral*/);
		this.alignElevatorAlgaeL2 = new Trigger(() -> (this.controller.a().getAsBoolean()))/* .and(RobotContainer.getInstance().driverOI.holdingCoral)*/;
		this.alignElevatorAlgaeL3 = new Trigger(() -> (this.controller.y().getAsBoolean()))/*.and(RobotContainer.getInstance().driverOI.holdingCoral)*/;
	}

	public final Trigger climbModeOn;
	public final Trigger setElevatorModeCoral;
	public final Trigger setElevatorModeAlgae;
	public final Trigger setElevatorModeNone;
	// public final Trigger climberOverrideLower;
	// public final Trigger climberOverrideRaise;
	// public final Trigger resetAngle;
	public final Trigger initializeClimber;
	// public final Trigger haultElevatpr;
	// public final Trigger raiseElevator;
	// public final Trigger lowerElevator;
	// public final Trigger intakeOut;
	// public final Trigger intakeIn;
	public final DoubleSupplier climbMotion;
	public final Trigger toggleReefHeightUp;
	public final Trigger toggleReefHeightDown;
	public final Trigger passOffCoral;
	public final Trigger alignElevatorCoral;
	public final Trigger alignElevatorAlgaeL2;
	public final Trigger alignElevatorAlgaeL3;
	public final Trigger homeElevator;  // sends the elevator back to home
	// public final Trigger halt;
	// public final Trigger slowedTrigger;

	public void configureControls() {
		// this.slowedTrigger.whileTrue(new JoystickDrive(RobotContainer.getInstance().drivetrain, 0.3));
		// this.halt.whileTrue(new RunCommand(() -> RobotContainer.getInstance().drivetrain.halt()));
		// haultElevatpr.onTrue(new InstantCommand(() -> RobotContainer.getInstance().elevator.setHaultMode()));
		setElevatorModeCoral.onTrue(new InstantCommand(() -> RobotContainer.getInstance().elevator.setElevatorMode(GamePieceType.CORAL)));
		setElevatorModeAlgae.onTrue(new InstantCommand(() -> RobotContainer.getInstance().elevator.setElevatorMode(GamePieceType.ALGAE)))
							.whileTrue(RobotContainer.getInstance().telePositionForAlgaeOverride());
		setElevatorModeNone.onTrue(new InstantCommand(() -> RobotContainer.getInstance().elevator.setElevatorMode(GamePieceType.NONE)));
	// this.climbModeOn.whileTrue(RobotContainer.getInstance().elevator.doClimb(this.climbMotion));
		this.toggleReefHeightDown.onTrue(new InstantCommand(RobotContainer.getInstance().elevator::toggleReefHeightDown));
		this.toggleReefHeightUp.onTrue(new InstantCommand(RobotContainer.getInstance().elevator::toggleReefHeightUp));
		this.passOffCoral.whileTrue(RobotContainer.getInstance().troughHandoffManual());
		// this.passOffCoral.whileTrue(RobotContainer.getInstance().troughHandoffAutomated());
		// TODO: change to toggle
		this.alignElevatorCoral.whileTrue(
			new ParallelCommandGroup(
				RobotContainer.getInstance().telePositionForCoralOveride(),
				RobotContainer.getInstance().reverseTrough()
			)
		);
		this.alignElevatorAlgaeL2.whileTrue(
			RobotContainer.getInstance().telePositionForAlgaeOverideL2()
		);
		this.alignElevatorAlgaeL3.whileTrue(
			RobotContainer.getInstance().telePositionForAlgaeOverideL3()
		);

		this.homeElevator.onTrue(new InstantCommand(() -> RobotContainer.getInstance().elevator.onEjectCoral(), RobotContainer.getInstance().elevator));
	}
}
//change elevator height
//create dashboard w/ shuffle board