package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
	public Elevator() {
		this.io = switch(Constants.mode) {
		case REAL -> new ElevatorIOReal(this);
		default -> throw new Error();
		};
	}

	public final ElevatorIO io;
	public final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

	@Override
	public void periodic() {
		this.io.updateInputs(this.inputs);
		Logger.processInputs("Elevator", this.inputs);
	}
}
