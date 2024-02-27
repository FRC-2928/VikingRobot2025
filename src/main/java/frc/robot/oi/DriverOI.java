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
import frc.robot.commands.shooter.ShootSpeaker;

public class DriverOI extends BaseOI {
	public DriverOI(final CommandXboxController controller) {
		super(controller);

		this.moveAxial = this.controller::getLeftY;
		this.moveLateral = this.controller::getLeftX;

		if(Constants.mode == Mode.REAL) {
			this.moveTheta = this.controller::getRightX;
			this.moveRotationX = this.controller::getRightX;
			this.moveRotationY = () -> -this.controller.getRightY();
		} else {
			this.moveTheta = () -> this.hid.getRawAxis(2);
			this.moveRotationX = () -> this.hid.getRawAxis(2);
			this.moveRotationY = () -> this.hid.getRawAxis(3);
		}

		this.slow = () -> MathUtil.interpolate(1, 0.5, this.controller.getRightTriggerAxis());

		this.shootFront = new Trigger(() -> this.controller.getRightTriggerAxis() > 0.5);
		this.shootRear = this.controller.rightBumper();

		this.lockWheels = this.controller.leftBumper();

		this.resetFOD = this.controller.y();
	}

	public final Supplier<Double> moveAxial;
	public final Supplier<Double> moveLateral;

	public final Supplier<Double> moveTheta;

	public final Supplier<Double> moveRotationX;
	public final Supplier<Double> moveRotationY;

	public final Supplier<Double> slow;

	public final Trigger shootFront;
	public final Trigger shootRear;

	public final Trigger lockWheels;

	public final Trigger resetFOD;

	public void configureControls() {
		this.shootFront
			.whileTrue(
				new ConditionalCommand(
					new ShootSpeaker(),
					new IntakeGround(true),
					() -> Robot.cont.shooter.inputs.holdingNote
				)
			);

		this.lockWheels.whileTrue(new LockWheels());

		this.resetFOD.onTrue(new InstantCommand(Robot.cont.drivetrain::resetAngle));
	}
}
