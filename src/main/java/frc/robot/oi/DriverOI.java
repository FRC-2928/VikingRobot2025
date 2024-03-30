package frc.robot.oi;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.Mode;
import frc.robot.commands.climber.Initialize;
import frc.robot.commands.drivetrain.LockWheels;
import frc.robot.commands.shooter.IntakeGround;
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

		this.shootSpeaker = new Trigger(() -> this.controller.getLeftTriggerAxis() > 0.5);
		this.shootAmp = this.controller.leftBumper();
		this.intakeShoot = new Trigger(() -> this.controller.getRightTriggerAxis() > 0.5);

		this.lockWheels = this.controller.rightBumper();

		this.resetFOD = this.controller.y();

		this.initializeClimber = this.controller.rightStick();

		this.moveAtSpeed = this.controller.x();
	}

	public final Supplier<Double> driveAxial;
	public final Supplier<Double> driveLateral;

	public final Supplier<Double> driveFORX;
	public final Supplier<Double> driveFORY;

	public final Trigger shootSpeaker;
	public final Trigger shootAmp;
	public final Trigger intakeShoot;

	public final Trigger lockWheels;

	public final Trigger resetFOD;

	public final Trigger initializeClimber;

	public final Trigger moveAtSpeed;

	public void configureControls() {
		this.shootSpeaker.whileTrue(new ShootSpeaker(true));
		this.shootAmp.whileTrue(new ShootAmp());
		this.intakeShoot.whileTrue(new IntakeGround());

		this.lockWheels.whileTrue(new LockWheels());

		this.resetFOD.onTrue(new InstantCommand(Robot.cont.drivetrain::resetAngle));

		this.initializeClimber.onTrue(new Initialize());

		this.moveAtSpeed
			.whileTrue(
				new RunCommand(
					() -> Robot.cont.drivetrain
						.control(
							new ChassisSpeeds(
								Units.MetersPerSecond.of(2),
								Units.MetersPerSecond.zero(),
								Units.RadiansPerSecond.of(0)
							)
						),
					Robot.cont.drivetrain
				)
			);
	}
}
