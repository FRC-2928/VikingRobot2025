package frc.robot.oi;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotContainer;
import frc.robot.Tuning;
import frc.robot.commands.drivetrain.CenterLimelight;
import frc.robot.commands.drivetrain.LockWheels;

public class DriverOI extends BaseOI {
	public final DoubleSupplier driveAxial;
	public final DoubleSupplier driveLateral;

	public final DoubleSupplier driveFORX;
	public final DoubleSupplier driveFORY;
	public final Trigger manualRotation;

	private final Trigger alignReefLeft;
	private final Trigger alignReefRight;
	private final Trigger alignReefCenter;
	private final Trigger alignReefCenterWithCoral;
	private final Trigger alignHP;
	private final Trigger alignProcessor;

	public final Trigger lockWheels;

	public final Trigger resetFOD;
	public final Trigger resetAngle;

	public final Trigger outputGamePiece;

	// Drivers asked for this control to be only for operator
	// public final Trigger toggleReefHeightUp;
	// public final Trigger toggleReefHeightDown;

	public final Trigger holdingCoral;

	// private final Trigger passOffCoral;

	public final Trigger closeToHP;
	public final Trigger closeToProcessor;
	public final Trigger inReefAlignRange;
	public final Trigger closeToReef;

	public final BooleanSupplier reefMovementLeft;
	public final BooleanSupplier reefMovementRight;
	public final BooleanSupplier reefMovementBack;
	public final BooleanSupplier reefMovementForward;
	private final BaseOI.Haptics haptics;

	public DriverOI(final CommandXboxController controller) {
		super(controller);

		this.driveAxial = this.controller::getLeftY;
		this.driveLateral = this.controller::getLeftX;
		this.haptics = new BaseOI.Haptics(this.hid);
		this.haptics.type = RumbleType.kBothRumble;
		this.haptics.interval = 1;
		this.haptics.dutyCycle = 1;
		this.haptics.powerTrue = 1;
		this.haptics.powerFalse = 0;

		this.holdingCoral = new Trigger(() -> (!RobotContainer.getInstance().intake.inputs.troughHasCoral && RobotContainer.getInstance().bananaFlywheels.holdingCoral()));

		if(Constants.mode == Mode.REAL) {
			this.driveFORX = this.controller::getRightX;
			this.driveFORY = () -> -this.controller.getRightY();
		} else {
			this.driveFORX = () -> this.hid.getRawAxis(2);
			this.driveFORY = () -> this.hid.getRawAxis(3);
		}
		this.manualRotation = this.controller.rightStick();

		this.closeToHP = new Trigger(() -> {return
			RobotContainer.getInstance().drivetrain.getEstimatedPosition().getTranslation().getDistance(Constants.hpBlueLeft) < Tuning.alignRadiusHP.get()
			|| RobotContainer.getInstance().drivetrain.getEstimatedPosition().getTranslation().getDistance(Constants.hpBlueRight) < Tuning.alignRadiusHP.get()
			|| RobotContainer.getInstance().drivetrain.getEstimatedPosition().getTranslation().getDistance(Constants.hpRedLeft) < Tuning.alignRadiusHP.get()
			|| RobotContainer.getInstance().drivetrain.getEstimatedPosition().getTranslation().getDistance(Constants.hpRedRight) < Tuning.alignRadiusHP.get();});
		this.closeToProcessor = new Trigger(() -> {return
			RobotContainer.getInstance().drivetrain.getEstimatedPosition().getTranslation().getDistance(Constants.processorBlue) < Tuning.alignRadiusProcessor.get()
			|| RobotContainer.getInstance().drivetrain.getEstimatedPosition().getTranslation().getDistance(Constants.processorRed) < Tuning.alignRadiusProcessor.get();});
		this.inReefAlignRange = new Trigger(() -> {return
			RobotContainer.getInstance().drivetrain.getEstimatedPosition().getTranslation().getDistance(Constants.blueReefCenter) < Tuning.alignRadiusReef.get()
			|| RobotContainer.getInstance().drivetrain.getEstimatedPosition().getTranslation().getDistance(Constants.redReefCenter) < Tuning.alignRadiusReef.get();});

		this.closeToReef = new Trigger(() -> {
			return RobotContainer.getInstance().drivetrain.getEstimatedPosition().getTranslation().getDistance(Constants.blueReefCenter) < Tuning.closeToReefRadius.get()
			|| RobotContainer.getInstance().drivetrain.getEstimatedPosition().getTranslation().getDistance(Constants.redReefCenter) < Tuning.closeToReefRadius.get();
		});

		this.alignReefLeft = this.controller.leftBumper()
							/* .and(this.holdingCoral)*/
							.and(this.inReefAlignRange);
		this.alignReefRight = this.controller.rightBumper()
							/* .and(this.holdingCoral)*/
							.and(this.inReefAlignRange);
		this.alignHP = (this.controller.leftBumper().or(this.controller.rightBumper()))
							/*.and(this.holdingCoral.negate())*/
							.and(this.closeToHP);
		this.alignProcessor = (this.controller.leftBumper().or(this.controller.rightBumper()))
							.and(this.holdingCoral.negate())
							.and(this.closeToProcessor);  // TODO: figure out the "holding" states...
							/*.and(this.nearProcessor());*/
		this.alignReefCenter = (this.controller.leftTrigger())
							.and(this.holdingCoral.negate())
							.and(this.inReefAlignRange);
							/*.and(this.nearReef())*/
		this.alignReefCenterWithCoral = (this.controller.leftTrigger())
							.and(this.holdingCoral)
							.and(this.inReefAlignRange);

		this.resetFOD = this.controller.y();
		this.resetAngle = this.controller.b().or(() -> DriverStation.isDisabled());

		this.outputGamePiece = this.controller.rightTrigger();

		this.reefMovementLeft = this.controller.povLeft();
		this.reefMovementRight = this.controller.povRight();
		this.reefMovementForward = this.controller.povUp();
		this.reefMovementBack = this.controller.povDown();

		this.lockWheels = this.controller.x();
		// Drivers asked for this control to be only for operator

		// this.passOffCoral = this.controller.a();
	}

	public BaseOI.Haptics getHaptics() {
		return this.haptics;
	}

	public void configureControls() {

		this.lockWheels.whileTrue(new LockWheels());
		this.resetFOD.onTrue(new InstantCommand(RobotContainer.getInstance().drivetrain::resetAngle));
		// TODO: update/change this with LL mode 3
		this.resetAngle
			.whileTrue(new RunCommand(RobotContainer.getInstance().drivetrain::seedLimelightImu))
			.whileFalse(new RunCommand(RobotContainer.getInstance().drivetrain::setImuMode2));
		this.alignReefLeft.whileTrue(
			RobotContainer.getInstance().telePositionForCoralLeft()
		).onFalse(
			RobotContainer.getInstance().raiseElevatorAtReef()
		);
		this.alignReefRight.whileTrue(
			RobotContainer.getInstance().telePositionForCoralRight()
		).onFalse(
			RobotContainer.getInstance().raiseElevatorAtReef()
		);
		this.alignReefCenter.whileTrue(
			RobotContainer.getInstance().telePositionForAlgae()
		).onFalse(
			RobotContainer.getInstance().pullAlgaeOffReef()
		);
		this.alignReefCenterWithCoral.whileTrue(
			new SequentialCommandGroup(
				CenterLimelight.centerLimelightCenter(),
				RobotContainer.getInstance().drivetrain.slowMode()
			)
		);
		this.alignHP.whileTrue(
			new SequentialCommandGroup(
				CenterLimelight.centerLimelightClosestHP(),
				RobotContainer.getInstance().drivetrain.slowMode()
			)
		);
		this.alignProcessor.whileTrue(
			Commands.sequence(
				CenterLimelight.centerLimelightProcessor(),
				new InstantCommand(RobotContainer.getInstance().elevator::onEjectAlgae)));
		this.outputGamePiece.whileTrue(RobotContainer.getInstance().bananaFlywheels.outputForward())
							.onFalse(new InstantCommand(() -> RobotContainer.getInstance().elevator.onEjectCoral(), RobotContainer.getInstance().elevator));
		// this.passOffCoral.whileTrue(RobotContainer.getInstance().troughHandoffManual());
		// Drivers asked for this control to be only for operator
		// this.toggleReefHeightDown.onTrue(new InstantCommand(RobotContainer.getInstance().elevator::toggleReefHeightDown));
		// this.toggleReefHeightUp.onTrue(new InstantCommand(RobotContainer.getInstance().elevator::toggleReefHeightUp));
	}
}
