package frc.robot.subsystems.elevator.io;

import frc.robot.subsystems.elevator.ElevatorState;

public interface ElevatorIO {
    public ElevatorState.InputState getState();
    public void setState(ElevatorState.OutputState output);
}
