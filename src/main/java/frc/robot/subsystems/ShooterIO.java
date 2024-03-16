package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.*;

public interface ShooterIO {
	@AutoLog
	public static class ShooterIOInputs {
		public Measure<Angle> angle = Units.Radians.zero();
		public boolean holdingNote;
		public Measure<Velocity<Angle>> flywheelSpeedA;
		public Measure<Velocity<Angle>> flywheelSpeedB;
	}

	public static enum Demand {
		Reverse(-1), Halt(0), Forward(1);

		private Demand(final double dir) { this.dir = dir; }

		public final double dir;
	}

	public default void rotate(final Measure<Angle> target) {}

	public default void runFlywheels(final double demand) {}

	public default void runFlywheelsVelocity(final double demand) {}

	public default void runFeeder(final Demand demand) {}

	public default void runIntake(final Demand demand) {}

	public default void updateInputs(final ShooterIOInputs inputs) {}
}
