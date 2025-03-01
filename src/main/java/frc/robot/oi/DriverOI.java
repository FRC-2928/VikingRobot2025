package frc.robot.oi;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.GamePieceType;
import frc.robot.Constants.Mode;
import frc.robot.Robot;
import frc.robot.commands.drivetrain.LockWheels;

public class DriverOI extends BaseOI {
	public final Supplier<Double> driveAxial;
	public final Supplier<Double> driveLateral;

	public final Supplier<Double> driveFORX;
	public final Supplier<Double> driveFORY;
	public final Trigger manualRotation;

	public final Trigger alignReefLeft;
	public final Trigger alignReefRight;
	public final Trigger alignHP;
	public final Trigger alignProcessor;

	public final Trigger lockWheels;

	public final Trigger resetFOD;
	public final Trigger resetAngle;

	public final Trigger resetPoseLimelight;

	public final Trigger outputGamePiece;

	public final Trigger toggleReefHeightUp;
	public final Trigger toggleReefHeightDown;

	private final Trigger holdingCoral;

	public int targetScoringLevel;

	public DriverOI(final CommandXboxController controller) {
		super(controller);

		this.driveAxial = this.controller::getLeftY;
		this.driveLateral = this.controller::getLeftX;

		this.holdingCoral = new Trigger(() -> (Robot.cont.intake.inputs.troughHasCoral || Robot.cont.bananaFlywheels.holdingCoral()));

		if(Constants.mode == Mode.REAL) {
			this.driveFORX = this.controller::getRightX;
			this.driveFORY = () -> -this.controller.getRightY();
		} else {
			this.driveFORX = () -> this.hid.getRawAxis(2);
			this.driveFORY = () -> this.hid.getRawAxis(3);
		}
		this.manualRotation = this.controller.rightStick();

		this.alignReefLeft = this.controller.leftBumper().and(this.holdingCoral);
		this.alignReefRight = this.controller.rightBumper().and(this.holdingCoral);
		this.alignHP = (this.controller.leftBumper().or(this.controller.rightBumper()))
							.and(this.holdingCoral.negate());
		this.alignProcessor = (this.controller.leftBumper().or(this.controller.rightBumper()))
							.and(this.holdingCoral.negate());
							/*.and(this.nearProcessor());*/

		this.alignReefCenter = (this.controller.leftBumper().or(this.controller.rightBumper()))
							.and(this.holdingCoral.negate());
							/*.and(this.nearReef())*/

		this.resetFOD = this.controller.y();
		this.resetAngle = this.controller.back();

		this.resetPoseLimelight = this.controller.b();

		this.outputGamePiece = this.controller.a();

		this.lockWheels = this.controller.x();

		this.toggleReefHeightDown = this.controller.povDown();
		this.toggleReefHeightUp = this.controller.povUp();

		this.targetScoringLevel = 1;
	}

	public void configureControls() {

		this.lockWheels.whileTrue(new LockWheels());
		this.resetFOD.onTrue(new InstantCommand(Robot.cont.drivetrain::resetAngle));
		this.resetAngle.whileTrue(new RunCommand(Robot.cont.drivetrain::seedLimelightImu)).whileFalse(new RunCommand(Robot.cont.drivetrain::setImuMode2));
		this.resetPoseLimelight.onTrue(new InstantCommand(Robot.cont.drivetrain::resetLimelightPose));
		this.alignReefLeft.whileTrue(
			Commands.sequence(
				CenterLimelight.CenterLimelightLeft(),
				Robot.cont.elevator.goToReefHeight(GamePieceType.CORAL));
		this.alignReefRight.whileTrue(
			Commands.sequence(
				CenterLimelight.CenterLimelightRight()
				Robot.cont.elevator.goToReefHeight(GamePieceType.CORAL));
		this.alignProcessor.whileTrue(Robot.cont.elevator.processorAlgae());
		this.outputGamePiece.whileTrue(Robot.cont.bananaFlywheels.outputForward());
		this.toggleReefHeightDown.onTrue(new InstantCommand(Robot.cont.elevator::toggleReefHeightDown));
		this.toggleReefHeightUp.onTrue(new InstantCommand(Robot.cont.elevator::toggleReefHeightDown));
	}
}
