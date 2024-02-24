package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.*;

public interface ShooterIO {
	@AutoLog
	public static class ShooterIOInputs {
		public Measure<Angle> angle;
		public boolean holdingNote;
		public Measure<Velocity<Angle>> flywheelSpeed;
	}

	public default void updateInputs(final ShooterIOInputs inputs) {}
}
