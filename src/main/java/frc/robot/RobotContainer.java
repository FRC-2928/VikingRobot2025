package frc.robot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SerialPort.WriteBufferMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.oi.DriverOI;
import frc.robot.oi.OperatorOI;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Diagnostics;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightFX;
import frc.robot.subsystems.Shooter;
import static frc.robot.subsystems.LimelightFX.Behavior.*;

public class RobotContainer {
	public final LoggedDashboardChooser<Command> autonomousChooser;

	public final DriverOI driverOI = new DriverOI(new CommandXboxController(0));
	public final OperatorOI operatorOI = new OperatorOI(new CommandXboxController(1));

	public final Diagnostics diag;

	public final Drivetrain drivetrain;
	public final Shooter shooter;
	public final Climber climber;

	public final LimelightFX fx = new LimelightFX();
	public final LimelightFX.Module fxScreen = this.fx
		.module(LimelightFX.Module.Geometry.grid, LimelightFX.Module.Rotation.R0);
	public final LimelightFX.Module[] fxStrips = new LimelightFX.Module[] {
		// we have 8 strips, however since the strips are daisy chained and we have 2 directly attached, we splice them together as one strip
		this.fx.module(LimelightFX.Module.Geometry.strip.size(32, 1), LimelightFX.Module.Rotation.R0),
		this.fx.module(LimelightFX.Module.Geometry.strip.size(32, 1), LimelightFX.Module.Rotation.R0),

		this.fx.module(LimelightFX.Module.Geometry.strip.size(32, 1), LimelightFX.Module.Rotation.R0),
		this.fx.module(LimelightFX.Module.Geometry.strip.size(32, 1), LimelightFX.Module.Rotation.R0), };
	public final LimelightFX.Behavior<?> fxStateHoldingNote;

	@SuppressWarnings({ "resource" })
	public RobotContainer() {
		Robot.instance.container = this;
		Robot.cont = this;

		Tuning.flywheelVelocity.get(); // load the class to put the tuning controls on the dashboard

		this.diag = new Diagnostics();
		this.drivetrain = new Drivetrain();
		this.shooter = new Shooter();
		this.climber = new Climber();

		{
			final BlinkBehavior beh = (BlinkBehavior) this.fx
				.behavior(BlinkBehavior.class)
				.on(this.fxScreen, 0)
				.on(this.fxStrips, 0);

			beh.colorA.set(new LimelightFX.Color(255, 127, 0));
			beh.colorB.set(LimelightFX.Color.WHITE);

			beh.timeOnA.set(1.0);
			beh.timeOffA.set(0.25);
			beh.timeBetween.set(0.0);
			beh.timeOnB.set(1.0);
			beh.timeOffB.set(0.25);
			beh.timeRepeat.set(0.0);

			beh.blinkCountA.set(1);
			beh.blinkCountB.set(1);
			beh.repeatCount.set(0);

			beh.fadeInA.set(0.0);
			beh.fadeOutA.set(0.0);
			beh.fadeInB.set(0.0);
			beh.fadeOutB.set(0.0);

			this.fxStateHoldingNote = beh;
		}

		this.fx.selector(() -> {
			if(this.shooter.inputs.holdingNote) return this.fxStateHoldingNote;
			else return null;
		});

		if(Constants.LimelightFX.enabled) {
			this.fx.initialize(() -> {
				final SerialPort serial = new SerialPort(115200, SerialPort.Port.kUSB1);
				serial.reset();
				serial.setTimeout(1);
				serial.setWriteBufferMode(WriteBufferMode.kFlushOnAccess);
				return str -> {
					System.out.print("command: " + str);
					Logger.recordOutput("LLFX/Command", Timer.getFPGATimestamp() + ": " + str);
					return serial.writeString(str) != 0;
				};
			});
		}

		this.autonomousChooser = new LoggedDashboardChooser<>(
			"Autonomous Routine",
			Autonomous.createAutonomousChooser()
		);

		this.driverOI.configureControls();
		this.operatorOI.configureControls();

		this.diag.configureControls();
	}

	public Command getAutonomousCommand() { return this.autonomousChooser.get(); }
}
