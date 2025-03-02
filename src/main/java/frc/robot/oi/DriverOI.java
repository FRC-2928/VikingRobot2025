package frc.robot.oi;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Robot;
import frc.robot.Tuning;
import frc.robot.commands.drivetrain.CenterLimelight;
import frc.robot.commands.drivetrain.LockWheels;

public class DriverOI extends BaseOI {
	public final Supplier<Double> driveAxial;
	public final Supplier<Double> driveLateral;

	public final Supplier<Double> driveFORX;
	public final Supplier<Double> driveFORY;
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

	public final Trigger resetPoseLimelight;

	public final Trigger outputGamePiece;

	public final Trigger toggleReefHeightUp;
	public final Trigger toggleReefHeightDown;

	private final Trigger holdingCoral;

	private final Trigger intakeButton;
	private final Trigger passOffCoral;

	public final Trigger closeToHP;
	public final Trigger closeToProcessor;
	public final Trigger closeToReef;

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

		this.closeToHP = new Trigger(() -> {return
			Robot.cont.drivetrain.getEstimatedPosition().getTranslation().getDistance(Constants.hpBlueLeft) < Tuning.alignRadiusHP.get()
			|| Robot.cont.drivetrain.getEstimatedPosition().getTranslation().getDistance(Constants.hpBlueRight) < Tuning.alignRadiusHP.get()
			|| Robot.cont.drivetrain.getEstimatedPosition().getTranslation().getDistance(Constants.hpRedLeft) < Tuning.alignRadiusHP.get()
			|| Robot.cont.drivetrain.getEstimatedPosition().getTranslation().getDistance(Constants.hpRedRight) < Tuning.alignRadiusHP.get();});
		this.closeToProcessor = new Trigger(() -> {return
			Robot.cont.drivetrain.getEstimatedPosition().getTranslation().getDistance(Constants.processorBlue) < Tuning.alignRadiusProcessor.get()
			|| Robot.cont.drivetrain.getEstimatedPosition().getTranslation().getDistance(Constants.processorRed) < Tuning.alignRadiusProcessor.get();});
		this.closeToReef = new Trigger(() -> {return
			Robot.cont.drivetrain.getEstimatedPosition().getTranslation().getDistance(Constants.blueReefCenter) < Tuning.alignRadiusReef.get()
			|| Robot.cont.drivetrain.getEstimatedPosition().getTranslation().getDistance(Constants.redReefCenter) < Tuning.alignRadiusReef.get();});

		this.alignReefLeft = this.controller.leftBumper()
							.and(this.holdingCoral)
							.and(this.closeToReef);
		this.alignReefRight = this.controller.rightBumper()
							.and(this.holdingCoral)
							.and(this.closeToReef);
		this.alignHP = (this.controller.leftBumper().or(this.controller.rightBumper()))
							.and(this.holdingCoral.negate())
							.and(this.closeToHP);
		this.alignProcessor = (this.controller.leftBumper().or(this.controller.rightBumper()))
							.and(this.holdingCoral.negate())
							.and(this.closeToProcessor);  // TODO: figure out the "holding" states...
							/*.and(this.nearProcessor());*/
		this.alignReefCenter = (this.controller.leftTrigger())
							.and(this.holdingCoral.negate())
							.and(this.closeToReef);
							/*.and(this.nearReef())*/
		this.alignReefCenterWithCoral = (this.controller.leftTrigger())
							.and(this.holdingCoral)
							.and(this.closeToReef);

		this.resetFOD = this.controller.y();
		this.resetAngle = this.controller.back();

		this.resetPoseLimelight = this.controller.b();

		this.outputGamePiece = this.controller.a();

		this.lockWheels = this.controller.x();

		this.toggleReefHeightDown = this.controller.povDown();
		this.toggleReefHeightUp = this.controller.povUp();

		this.targetScoringLevel = 1;
		this.intakeButton = this.controller.povLeft();
		this.passOffCoral = new Trigger(() -> (Robot.cont.intake.holdingGamePeice() && Robot.cont.elevator.inputs.isElevatorHomed && Robot.cont.elevator.inputs.isPivotHomed && !Robot.cont.bananaFlywheels.holdingCoral()));
	}

	public void configureControls() {

		this.lockWheels.whileTrue(new LockWheels());
		this.resetFOD.onTrue(new InstantCommand(Robot.cont.drivetrain::resetAngle));
		// TODO: update/change this with LL mode 3
		this.resetAngle.whileTrue(new RunCommand(Robot.cont.drivetrain::seedLimelightImu)).whileFalse(new RunCommand(Robot.cont.drivetrain::setImuMode2));
		this.alignReefLeft.whileTrue(
			Robot.cont.telePositionForCoralLeft()
		);
		this.alignReefRight.whileTrue(
			Robot.cont.telePositionForCoralRight()
		);
		this.alignReefCenter.whileTrue(
			Robot.cont.telePositionForAlgae()
		);
		this.alignReefCenterWithCoral.whileTrue(
			new SequentialCommandGroup(
				CenterLimelight.centerLimelightCenter(),
				Robot.cont.drivetrain.slowDrive
			)
		);
		this.alignHP.whileTrue(
			new SequentialCommandGroup(
				CenterLimelight.centerLimelightClosestHP(),
				Robot.cont.drivetrain.slowDrive
			)
		);
		this.alignProcessor.whileTrue(
			Commands.sequence(
				CenterLimelight.centerLimelightProcessor(),
				Robot.cont.elevator.processorAlgae())); // what is this supposed to do...? seems worthless... except for depositing...
		this.outputGamePiece.whileTrue(Robot.cont.bananaFlywheels.outputForward());
		this.intakeButton.whileTrue(Robot.cont.intake.intakeTrough());
		this.passOffCoral.onTrue(Robot.cont.passCoral());
		this.toggleReefHeightDown.onTrue(new InstantCommand(() -> {
			this.targetScoringLevel = MathUtil.clamp(this.targetScoringLevel-1, 1, 4);
		}));
		this.toggleReefHeightUp.onTrue(new InstantCommand(() -> {
			this.targetScoringLevel = MathUtil.clamp(this.targetScoringLevel+1, 1, 4);
		}));
		this.toggleReefHeightDown.onTrue(new InstantCommand(Robot.cont.elevator::toggleReefHeightDown));
		this.toggleReefHeightUp.onTrue(new InstantCommand(Robot.cont.elevator::toggleReefHeightDown));
	}
}
