package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drivetrain.JoystickDrive;
import frc.robot.commands.drivetrain.LockWheels;
import frc.robot.oi.DriverOI;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ClimberIO;
import frc.robot.subsystems.ClimberIOReal;
import frc.robot.subsystems.Diagnostics;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.DrivetrainModifier;
import frc.robot.subsystems.DrivetrainModifier.Modification;
import frc.robot.subsystems.GyroIO;
import frc.robot.subsystems.GyroIOReal;
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
	public final DrivetrainModifier mod;
	public final Shooter shooter;
	public final Climber climber;

	public RobotContainer() {
		Robot.instance.container = this;
		Robot.cont = this;

		this.diag = new Diagnostics();
		this.drivetrain = new Drivetrain();
		this.shooter = new Shooter();
		this.climber = new Climber();

		this.mod = new DrivetrainModifier();
		this.mod.setDefaultCommand(new Modification() {
			@Override
			public ChassisSpeeds modify(final ChassisSpeeds control) { return control; }
		});

		this.autonomousChooser = new LoggedDashboardChooser<>(
			"Autonomous Routine",
			AutonomousRoutines.createAutonomousChooser()
		);

		this.configureDriverControls();

		this.diag.release.whileTrue(this.diag.new Release());
	}

	private void configureDriverControls() {
		this.driverOI.lockWheels.whileTrue(new LockWheels());

		this.driverOI.resetFOD.onTrue(new InstantCommand(() -> {
			this.drivetrain.resetFOD();
			this.drivetrain.resetAngle();
		}));
	}

	public void teleop() { this.drivetrain.setDefaultCommand(new JoystickDrive()); }

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() { return this.autonomousChooser.get(); }

}
