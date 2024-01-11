package frc.robot.subsystems.armtest.io;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.subsystems.armtest.ArmState.InputState;
import frc.robot.subsystems.armtest.ArmState.OutputState;

public class ArmRealIO implements ArmIO{

    Spark spark = new Spark(1);
    //Encoder encoder

    @Override
    public InputState getState() {
        //get encoder value from here
        return null;
    }

    @Override
    public void setState(OutputState output) {
        //spark.setVoltage(output.voltage());
    }
    
}
