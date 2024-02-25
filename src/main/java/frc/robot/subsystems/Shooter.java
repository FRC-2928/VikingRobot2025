package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
	public Shooter() {
		this.io = switch(Constants.mode) {
		case REAL -> new ShooterIOReal(this);
		default -> throw new Error();
		};
	}

	public final ShooterIO io;
	public final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

	public void apply(final Measure<Angle> angle) {
		// todo
	}

	@Override
	public void periodic() {
		this.io.updateInputs(this.inputs);
		Logger.processInputs("Climber", this.inputs);
	}
}
