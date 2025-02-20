package frc.robot.oi;

import java.util.function.Supplier;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Robot;
import frc.robot.commands.drivetrain.LockWheels;

public class DriverOI extends BaseOI {
	public DriverOI(final CommandXboxController controller) {
		super(controller);

		this.driveAxial = this.controller::getLeftY;
		this.driveLateral = this.controller::getLeftX;

		if(Constants.mode == Mode.REAL) {
			this.driveFORX = this.controller::getRightX;
			this.driveFORY = () -> -this.controller.getRightY();
		} else {
			this.driveFORX = () -> this.hid.getRawAxis(2);
			this.driveFORY = () -> this.hid.getRawAxis(3);
		}
		this.manualRotation = this.controller.rightStick();

		this.shootSpeaker = this.controller.leftTrigger();
		this.shootAmp = this.controller.leftBumper();
		this.intake = this.controller.rightTrigger();

		this.ferry = this.controller.rightBumper();

		this.resetFOD = this.controller.y();

		this.resetPoseLimelight = this.controller.a();
		this.moveElevatorUp = this.controller.a();

		this.lockWheels = this.controller.x();
	}

	public final Supplier<Double> driveAxial;
	public final Supplier<Double> driveLateral;

	public final Supplier<Double> driveFORX;
	public final Supplier<Double> driveFORY;
	public final Trigger manualRotation;

	public final Trigger shootSpeaker;
	public final Trigger shootAmp;
	public final Trigger intake;

	public final Trigger lockWheels;

	public final Trigger resetFOD;

	public final Trigger resetPoseLimelight;
	public final Trigger moveElevatorUp;

	public final Trigger ferry;

	public void configureControls() {

		this.lockWheels.whileTrue(new LockWheels());
		this.resetFOD.onTrue(new InstantCommand(Robot.cont.drivetrain::resetAngle));
		// this.resetPoseLimelight.onTrue(new InstantCommand(Robot.cont.drivetrain::resetLimelightPose));
		this.moveElevatorUp
		.whileTrue(new RunCommand(() -> {
			Robot.cont.elevator.moveToPosition(Units.Feet.of(5));
		}, Robot.cont.elevator))
		.whileFalse(new RunCommand(() -> {
			Robot.cont.elevator.moveToPosition(Units.Feet.of(1));
		}, Robot.cont.elevator));
	}
}
