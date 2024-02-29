package frc.robot.oi;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.Mode;
import frc.robot.commands.drivetrain.LockWheels;
import frc.robot.commands.shooter.IntakeGround;
import frc.robot.commands.shooter.IntakeSource;
import frc.robot.commands.shooter.ShootAmp;
import frc.robot.commands.shooter.ShootSpeaker;

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

		this.slow = () -> MathUtil.interpolate(1, 0.5, this.controller.getLeftTriggerAxis());

		this.aimFront = new Trigger(() -> this.controller.getRightTriggerAxis() > 0.1);
		this.shootFront = new Trigger(() -> this.controller.getRightTriggerAxis() > 0.9);
		this.aimRear = new Trigger(() -> this.controller.getLeftTriggerAxis() > 0.1);
		this.shootAmp = new Trigger(() -> this.controller.getLeftTriggerAxis() > 0.9);

		this.lockWheels = this.controller.leftBumper();

		this.resetFOD = this.controller.y();
	}

	public final Supplier<Double> driveAxial;
	public final Supplier<Double> driveLateral;

	public final Supplier<Double> driveFORX;
	public final Supplier<Double> driveFORY;

	public final Supplier<Double> slow;

	public final Trigger aimFront;
	public final Trigger shootFront;
	public final Trigger aimRear;
	public final Trigger shootAmp;

	public final Trigger lockWheels;

	public final Trigger resetFOD;

	public void configureControls() {
		this.aimFront
			.whileTrue(
				new ConditionalCommand(
					new ShootSpeaker(),
					new IntakeGround(true),
					() -> Robot.cont.shooter.inputs.holdingNote
				)
			);
		this.aimRear
			.whileTrue(
				new ConditionalCommand(new ShootAmp(), new IntakeSource(), () -> Robot.cont.shooter.inputs.holdingNote)
			);

		this.lockWheels.whileTrue(new LockWheels());

		this.resetFOD.onTrue(new InstantCommand(Robot.cont.drivetrain::resetAngle));
	}
}
