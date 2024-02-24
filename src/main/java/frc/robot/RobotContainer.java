package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drivetrain.JoystickDrive;
import frc.robot.commands.drivetrain.LockWheels;
import frc.robot.oi.DriverOI;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ClimberIO;
import frc.robot.subsystems.ClimberIOReal;
import frc.robot.subsystems.Diagnostics;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.GyroIO;
import frc.robot.subsystems.GyroIOPigeon2;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.ModuleIO;
import frc.robot.subsystems.ModuleIOSim;
import frc.robot.subsystems.ModuleIOReal;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterIO;
import frc.robot.subsystems.ShooterIOReal;

public class RobotContainer {
	public final LoggedDashboardChooser<Command> autonomousChooser;

	public final DriverOI driverOI = new DriverOI(new CommandXboxController(0));
	// public final OperatorOI operatorOI = new OperatorOI(new CommandXboxController(1));

	public final Diagnostics diag;

	public final Drivetrain drivetrain;
	public final Shooter shooter;
	public final Climber climber;

	public RobotContainer() {
		Robot.instance.container = this;
		Robot.cont = this;

		switch(Constants.mode) {
		case REAL -> {
			this.diag = new Diagnostics();

			this.drivetrain = new Drivetrain(
				new GyroIOPigeon2(),
				new ModuleIOReal(SwerveModule.Place.FrontLeft),
				new ModuleIOReal(SwerveModule.Place.FrontRight),
				new ModuleIOReal(SwerveModule.Place.BackLeft),
				new ModuleIOReal(SwerveModule.Place.BackRight)
			);
			this.shooter = new Shooter(new ShooterIOReal());
			this.climber = new Climber(new ClimberIOReal());
		}
		case SIM -> {
			this.diag = null;

			this.drivetrain = new Drivetrain(
				null,
				new ModuleIOSim(),
				new ModuleIOSim(),
				new ModuleIOSim(),
				new ModuleIOSim()
			);
			this.shooter = null;
			this.climber = null;
		}
		case REPLAY -> {
			this.diag = null;

			this.drivetrain = new Drivetrain(new GyroIO() {
			}, new ModuleIO() {
			}, new ModuleIO() {
			}, new ModuleIO() {
			}, new ModuleIO() {
			});
			this.shooter = new Shooter(new ShooterIO() {

			});
			this.climber = new Climber(new ClimberIO() {
			});
		}
		default -> {
			throw new Error();
		}
		}
		;

		this.autonomousChooser = new LoggedDashboardChooser<>(
			"Autonomous Routine",
			AutonomousRoutines.createAutonomousChooser(this.drivetrain)
		);

		this.configureDriverControls();

		this.diag.release.whileTrue(this.diag.new Release());
	}

	private void configureDriverControls() {
		this.driverOI.resetFOD.onTrue(new RunCommand(this.drivetrain::reset)); // Y Button
		this.driverOI.lock.whileTrue(new LockWheels()); // Left Bumper
	}

	public void teleop() { this.drivetrain.setDefaultCommand(new JoystickDrive()); }

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() { return this.autonomousChooser.get(); }

}
