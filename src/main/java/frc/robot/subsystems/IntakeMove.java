// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeMove extends SubsystemBase {

  public CANSparkMax intakeMoveLeft;
  public AbsoluteEncoder m_AbsoluteEncoderLeft;

  public CANSparkMax intakeMoveRight;
  public AbsoluteEncoder m_AbsoluteEncoderRight;
  //private RelativeEncoder wristEncoder;

  private SparkPIDController intakeMoveLeftPidController;
  private SparkPIDController intakeMoveRightPidController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  //private double maxSpeed = 0.25;
  //private double deadBand = 0.1;

  /** Creates a new Intake. */
  public IntakeMove() {
    intakeMoveLeft = new CANSparkMax(Constants.RobotConstants.intakeMoveLeftCANID, MotorType.kBrushless);
    intakeMoveLeft.restoreFactoryDefaults();

    intakeMoveLeftPidController = intakeMoveLeft.getPIDController();
    m_AbsoluteEncoderLeft = intakeMoveLeft.getAbsoluteEncoder(Type.kDutyCycle);
    intakeMoveLeftPidController.setFeedbackDevice(m_AbsoluteEncoderLeft);

    intakeMoveRight = new CANSparkMax(Constants.RobotConstants.intakeMoveRightCANID, MotorType.kBrushless);
    intakeMoveRight.restoreFactoryDefaults();

    intakeMoveRightPidController = intakeMoveRight.getPIDController();
    m_AbsoluteEncoderRight = intakeMoveRight.getAbsoluteEncoder(Type.kDutyCycle);
    intakeMoveRightPidController.setFeedbackDevice(m_AbsoluteEncoderRight);


    //m_AbsoluteEncoder.setPositionConversionFactor(360);
    //m_AbsoluteEncoder.setVelocityConversionFactor(1);
    intakeMoveLeft.setInverted(true);
    intakeMoveLeft.setIdleMode(IdleMode.kBrake);
    m_AbsoluteEncoderLeft.setZeroOffset(0.6526145);

    intakeMoveRight.setInverted(true);
    intakeMoveRight.setIdleMode(IdleMode.kBrake);
    m_AbsoluteEncoderRight.setZeroOffset(0.6526145);

    intakeMoveLeft.setSmartCurrentLimit(45);
    intakeMoveRight.setSmartCurrentLimit(45);

    kP = 2.8; //2.5 last working value
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0;
    kMaxOutput = .5;
    kMinOutput = -.5;

    intakeMoveLeftPidController.setP(kP);
    intakeMoveLeftPidController.setI(kI);
    intakeMoveLeftPidController.setD(kD);
    intakeMoveLeftPidController.setIZone(kIz);
    intakeMoveLeftPidController.setFF(kFF);
    intakeMoveLeftPidController.setOutputRange(kMinOutput, kMaxOutput);

    intakeMoveLeftPidController.setPositionPIDWrappingEnabled(true);
    intakeMoveLeftPidController.setPositionPIDWrappingMinInput(0);
    intakeMoveLeftPidController.setPositionPIDWrappingMaxInput(1);

    intakeMoveRightPidController.setP(kP);
    intakeMoveRightPidController.setI(kI);
    intakeMoveRightPidController.setD(kD);
    intakeMoveRightPidController.setIZone(kIz);
    intakeMoveRightPidController.setFF(kFF);
    intakeMoveRightPidController.setOutputRange(kMinOutput, kMaxOutput);

    intakeMoveRightPidController.setPositionPIDWrappingEnabled(true);
    intakeMoveRightPidController.setPositionPIDWrappingMinInput(0);
    intakeMoveRightPidController.setPositionPIDWrappingMaxInput(1);
  }

  @Override

  public void periodic() {
    SmartDashboard.putNumber("Intake Position(Left)", getAbsoluteEncoderPositionLeft());
    SmartDashboard.putNumber("Intake Position(Right)", getAbsoluteEncoderPositionRight());
    //SmartDashboard.putNumber("Wrist Joystick", RobotContainer.operator.getRawAxis(5));
    //SmartDashboard.putNumber("Wrist encoder", wristMotor.getEncoder().getPosition());

    /* if (RobotContainer.operator.getRawAxis(5) > deadBand) {
      wristMotor.set(-RobotContainer.operator.getRawAxis(5) * maxSpeed);
    } else if (RobotContainer.operator.getRawAxis(5) < -deadBand) {
      wristMotor.set(-RobotContainer.operator.getRawAxis(5) * maxSpeed);
    } */



    /* if (RobotContainer.operator.getRawButtonPressed(1)) {
      intakeMotor.set(1);
    }
      else if (RobotContainer.operator.getRawButtonReleased(1)) {
        intakeMotor.set(0);
      }

      if (RobotContainer.operator.getRawButtonPressed(2)) {
        intakeMotor.set(-1);
      }
      else if (RobotContainer.operator.getRawButtonReleased(2)) {
        intakeMotor.set(0);
      } */


    }




  public void setSpeed(double speed)
  {
    intakeMoveLeft.set(speed);
    intakeMoveRight.set(speed);
  }

  public void stop()
  {
    intakeMoveLeft.stopMotor();
    intakeMoveRight.stopMotor();
  }


  public double getAbsoluteEncoderPositionLeft() {
    return m_AbsoluteEncoderLeft.getPosition();
  }

  public double getAbsoluteEncoderPositionRight() {
    return m_AbsoluteEncoderRight.getPosition();
  }

  public void setPositionLeft(double position) {
    intakeMoveLeftPidController.setReference(position, CANSparkMax.ControlType.kPosition);
  }

  public void setPositionRight(double position) {
    intakeMoveRightPidController.setReference(position, CANSparkMax.ControlType.kPosition);
  }

}
