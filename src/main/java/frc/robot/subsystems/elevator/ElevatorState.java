package frc.robot.subsystems.elevator;

import java.util.Optional;

public class ElevatorState {
    public record InputState(
        double currentHeightMeters,
        double currentVelocityMetersPerSecond
    ) {}

    public record OutputState(
        Optional<Double> voltage
    ) {}

    public record GoalState(
        double position
    ) {}
}
