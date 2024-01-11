package frc.robot.subsystems.armtest.io;

import frc.robot.subsystems.armtest.ArmState;

public interface ArmIO {
    public ArmState.InputState getState();
    public void setState(ArmState.OutputState output);
}