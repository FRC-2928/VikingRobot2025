package frc.robot.oi;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public abstract class BaseOI {
	public static final class Haptics {
		private double dutyCycle;
		// todo: implement a duty cycle of haptics, add to lockwheels, fod reset, (intaking && game piece detected inside robot)
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
