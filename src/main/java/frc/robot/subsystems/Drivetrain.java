package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Drivetrain extends SubsystemBase {
	private final double width;
	private final double length;
	private double speedMultiplier = 1;

	public Drivetrain(double width, double length) {
		this.width = width;
		this.length = length;
	}

	public final double getWidth() {
		return width;
	}

	public final double getLength() {
		return length;
	}

	public double getSpeedMultiplier() {
		return speedMultiplier;
	}

	public void setSpeedMultiplier(double speedMultiplier) {
		this.speedMultiplier = speedMultiplier;
	}

	public abstract double getMaxAcceleration();

	public abstract double getMaxVelocity();
}
