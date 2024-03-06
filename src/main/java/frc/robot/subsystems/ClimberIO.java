package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
	@AutoLog
	public class ClimberIOInputs {
		public double position;
		public boolean home;
	}

	public default void set(final double position, final boolean fast) {}

	public default void offset(final double offset, final boolean fast) {}

	public default void overrideLock(final boolean engaged) {}

	public default void updateInputs(final ClimberIOInputs inputs) {}

	public default void periodic() {}
}
