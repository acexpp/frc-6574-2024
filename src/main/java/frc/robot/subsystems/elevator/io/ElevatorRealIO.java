package frc.robot.subsystems.elevator.io;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.subsystems.elevator.ElevatorState.InputState;
import frc.robot.subsystems.elevator.ElevatorState.OutputState;

public class ElevatorRealIO implements ElevatorIO {

    Spark spark = new Spark(1);
    // Encoder encoder

    @Override
    public InputState getState() {
        // get encoder value return here
        return null;
    }

    @Override
    public void setState(OutputState output) {
        // spark.setVoltage(output.voltage());
    }
    
}
