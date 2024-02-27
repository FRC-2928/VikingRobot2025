package frc.robot.oi;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public abstract class BaseOI {
	public static final class Haptics {
		public Haptics(final XboxController hid) { this.hid = hid; }

		private final XboxController hid;

		public RumbleType type;
		public long interval;
		public double dutyCycle;
		public double powerTrue;
		public double powerFalse;

		public void update() {
			// todo: implement a duty cycle of haptics, add to lockwheels, fod reset, (intaking && game piece detected inside robot)
			this.hid
				.setRumble(
					this.type,
					(Logger.getRealTimestamp() % this.interval / (double) this.interval) <= this.dutyCycle
						? this.powerTrue
						: this.powerFalse
				);
		}

		public void stop() { this.hid.setRumble(RumbleType.kBothRumble, 0); }
	}

	public CommandXboxController controller;
	public final XboxController hid;

	/**
	 * C-Stop, the most useful command you'll ever have Press this magical button (start/left "window" button) to stop *all running commands*.
	 *
	 * @implNote If you remove this command I will haunt you forever
	 */
	public final Trigger cstop;

	protected BaseOI(final CommandXboxController controller) {
		this.controller = controller;
		this.hid = controller.getHID();

		this.cstop = this.controller.start();
	}
}
