// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

public interface ModuleIO {
	@AutoLog
	public static class ModuleIOInputs {
		public Measure<Angle> drivePosition = Units.Degrees.zero();
		public Measure<Velocity<Distance>> driveVelocity = Units.MetersPerSecond.zero();
		public Measure<Current> driveCurrent = Units.Amps.zero();
		public Measure<Voltage> driveVoltage = Units.Volts.zero();

		public Measure<Angle> angle = Units.Degrees.zero();
		public Measure<Current> azimuthCurrent = Units.Amps.zero();
		public Measure<Voltage> azimuthVoltage = Units.Volts.zero();
	}

	public default void setDriveVoltage(final double volts) {}

	public default void setAzimuthVoltage(final double volts) {}

	public default void updateInputs(final ModuleIOInputs inputs) {}
}
