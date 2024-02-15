// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  public TalonFX kIntakeShooterLeft;
  public TalonFX kIntakeShooterRight;
  public TalonFXConfiguration shooterAngleFXConfig = new TalonFXConfiguration();
  public CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs();

  
  /** Creates a new Shooter. */
  public Shooter() {
    kIntakeShooterLeft = new TalonFX(Constants.RobotConstants.shooterLeftCANID);
    kIntakeShooterLeft.getConfigurator().apply(shooterAngleFXConfig);
    kIntakeShooterLeft.setNeutralMode(NeutralModeValue.Brake);
    kIntakeShooterLeft.getConfigurator().apply(currentLimitConfig.withStatorCurrentLimit(25));

    kIntakeShooterRight = new TalonFX(Constants.RobotConstants.shooterRightCANID);
    kIntakeShooterRight.getConfigurator().apply(shooterAngleFXConfig);
    kIntakeShooterRight.setNeutralMode(NeutralModeValue.Brake);
    kIntakeShooterRight.getConfigurator().apply(currentLimitConfig.withStatorCurrentLimit(25));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setShooterSpeed(double speed) {
    kIntakeShooterLeft.set(-speed);
    kIntakeShooterRight.set(speed);
  }
}
