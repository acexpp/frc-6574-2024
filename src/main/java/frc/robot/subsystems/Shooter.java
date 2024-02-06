// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  public CANSparkMax kIntakeShooterLeft;
  public CANSparkMax kIntakeShooterRight;
  
  /** Creates a new Intake. */
  public Shooter() {
    kIntakeShooterLeft = new CANSparkMax(Constants.RobotConstants.shooterLeftCANID, MotorType.kBrushless);
    kIntakeShooterLeft.restoreFactoryDefaults();
    kIntakeShooterLeft.setIdleMode(IdleMode.kBrake);
    kIntakeShooterLeft.setSmartCurrentLimit(25);

    kIntakeShooterRight = new CANSparkMax(Constants.RobotConstants.shooterRightCANID, MotorType.kBrushless);
    kIntakeShooterRight.restoreFactoryDefaults();
    kIntakeShooterRight.setIdleMode(IdleMode.kBrake);
    kIntakeShooterRight.setSmartCurrentLimit(25);
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
