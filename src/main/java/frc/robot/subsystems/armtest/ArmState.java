package frc.robot.subsystems.armtest;

import java.util.Optional;

public class ArmState {
    public record InputState(
        double currentAngleDegrees,
        double currentVelocityDegreesPerSecond
    ) {}

    public record OutputState(
        Optional<Double> voltage
    ) {}

    public record GoalState(
        double position
    ) {}
    
}
