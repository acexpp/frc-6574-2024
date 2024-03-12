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

  public TalonFX kShooterLeft;
  public TalonFX kShooterRight;
  public TalonFXConfiguration shooterAngleFXConfig = new TalonFXConfiguration();
  public CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs();

  
  /** Creates a new Shooter. */
  public Shooter() {
    kShooterLeft = new TalonFX(Constants.RobotConstants.shooterLeftCANID);
    kShooterLeft.getConfigurator().apply(shooterAngleFXConfig);
    kShooterLeft.setNeutralMode(NeutralModeValue.Brake);
    kShooterLeft.getConfigurator().apply(currentLimitConfig.withStatorCurrentLimit(25));

    kShooterRight = new TalonFX(Constants.RobotConstants.shooterRightCANID);
    kShooterRight.getConfigurator().apply(shooterAngleFXConfig);
    kShooterRight.setNeutralMode(NeutralModeValue.Brake);
    kShooterRight.getConfigurator().apply(currentLimitConfig.withStatorCurrentLimit(25));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setShooterSpeed(double speed) {
    kShooterLeft.set(-speed);
    kShooterRight.set(-speed);
  }
}
