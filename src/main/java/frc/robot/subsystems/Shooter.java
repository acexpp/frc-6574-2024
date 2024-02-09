// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//THIS SUBSYTEM USES KRAKENS - FIGHT PHOENIX API >:((
package frc.robot.subsystems;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  public TalonFX kIntakeShooterLeft;
  public TalonFX kIntakeShooterRight;
  public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();

  
  /** Creates a new Intake. */
  public Shooter() {
    kIntakeShooterLeft = new TalonFX(Constants.RobotConstants.shooterLeftCANID);
    kIntakeShooterLeft.getConfigurator().apply(new TalonFXConfiguration());
    kIntakeShooterLeft.setNeutralMode(NeutralModeValue.Brake);
    //FIGURE OUT HOW TO SET CURRENT LIMIT >:((

    kIntakeShooterRight = new TalonFX(Constants.RobotConstants.shooterLeftCANID);
    kIntakeShooterRight.getConfigurator().apply(new TalonFXConfiguration());
    kIntakeShooterRight.setNeutralMode(NeutralModeValue.Brake);
    //FIGURE OUT HOW TO SET CURRENT LIMIT >:((
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setIntakeSpeed(double speed) {
    kIntakeShooterLeft.set(speed);
    kIntakeShooterRight.set(speed);

  }

  public void setOutakeSpeed() {
    kIntakeShooterLeft.set(-1);
    kIntakeShooterRight.set(-1);

  }
}
