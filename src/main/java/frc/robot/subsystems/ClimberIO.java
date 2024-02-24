package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.*;

public interface ClimberIO {
	@AutoLog
	public class ClimberIOInputs {
		public double ticks;
		public boolean limit;
	}

	public default void drive(final Measure<Velocity<Distance>> velocity) {}

	public default void zero() {}

	public default void updateInputs(final ClimberIOInputs inputs) {}
}
