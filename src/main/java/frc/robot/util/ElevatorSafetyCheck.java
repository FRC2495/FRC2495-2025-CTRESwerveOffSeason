// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.function.BooleanSupplier;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Neck;

/** The purpose of this class is to check if it's okay to close the mouth.
 * 
 * It's the case when the neck is not in the danger zone.
 */
public class ElevatorSafetyCheck implements BooleanSupplier
{
	private Elevator elevator;

	public ElevatorSafetyCheck(Elevator elevator) {
		this.elevator = elevator;
	}

	@Override
	public boolean getAsBoolean() {
		return !elevator.isDangerous();
	}
}
