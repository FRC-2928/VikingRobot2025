package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.LimelightFX.*;
import frc.robot.subsystems.LimelightFX.Behavior.*;
import frc.robot.subsystems.LimelightFX.Module.*;

public class LimelightFXManager {
	public final LimelightFX fx = new LimelightFX();

	public final LimelightFX.Module grid = this.fx.module(Geometry.grid, Rotation.R0);
	public final LimelightFX.Module[] strips = new LimelightFX.Module[] {
		// we have 8 strips, however since the strips are daisy chained and we have 2 directly attached, we splice them together as one strip
		this.fx.module(Geometry.strip.size(16, 1), Rotation.R0),
		this.fx.module(Geometry.strip.size(16, 1), Rotation.R0),
		this.fx.module(Geometry.strip.size(16, 1), Rotation.R0),
		this.fx.module(Geometry.strip.size(16, 1), Rotation.R0),

		this.fx.module(Geometry.strip.size(16, 1), Rotation.R0),
		this.fx.module(Geometry.strip.size(16, 1), Rotation.R0),
		this.fx.module(Geometry.strip.size(16, 1), Rotation.R0),
		this.fx.module(Geometry.strip.size(16, 1), Rotation.R0), };

	public final Behavior<
		?> behDisabledStrips = this.fx.behavior(SolidColorBehavior.class).cfg(beh -> beh.color.set(Color.WHITE));
	public final Behavior<?> behDisabledGrid = this.fx.behavior(ImageBehavior.class).of("disabled");

	public final Behavior<?> behDisabledDiagnosticsIssue = this.fx
		.behavior(SolidColorBehavior.class)
		.cfg(beh -> beh.color.set(Color.RED));

	public final Behavior<?> behAuto = this.fx.behavior(BlinkBehavior.class);
	public final Behavior<?> behTeleop = this.fx.behavior(ImageBehavior.class).of("teleop");
	public final Behavior<?> behHoldingNote = this.fx
		.behavior(SolidColorBehavior.class)
		.cfg(beh -> beh.color.set(Constants.LimelightFX.Colors.note))
		.on(this.grid, 0)
		.on(this.strips, 0);

	@SuppressWarnings({ "resource" })
	public LimelightFXManager() {
		if(!Constants.LimelightFX.enabled) return;

		this.fx.selector(() -> {
			if(Robot.cont.shooter.inputs.holdingNote) return this.behHoldingNote;
			else return null;
		});

		this.fx.initialize(() -> {
			final SerialPort serial = new SerialPort(115200, SerialPort.Port.kUSB1);
			//serial.setWriteBufferMode(WriteBufferMode.kFlushOnAccess);
			//Logger.recordOutput("LLFX/Command", Timer.getFPGATimestamp() + ": " + str);
			return str -> {
				System.out.println("llfx: '" + str + "'");
				return serial.writeString(str + '\n') != 0;
			};
		});

		this.fx.sound(SystemSound.Beep2, 0, 0, 1);
	}
}
