package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
	public Climber() {
		this.io = switch(Constants.mode) {
		case REAL -> new ClimberIOReal();
		default -> throw new Error();
		};
	}

	public final ClimberIO io;
	public final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

	private final PIDController pid = Constants.Climber.pid.createController();

	public void apply(final Measure<Distance> position) {
		// todo
	}
}
