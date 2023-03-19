// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.IntakeConstants.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final CANSparkMax mMotor;
  private State mCurrentState;
  public Intake() {
    mMotor = new CANSparkMax(6, MotorType.kBrushless);
    mMotor.setSmartCurrentLimit(15);
    mMotor.setInverted(false);
    mMotor.enableVoltageCompensation(RobotConstants.maxVoltage);
    mMotor.burnFlash();
    mCurrentState = State.STARTING;
  }

  private void runIntake(){
    mMotor.set(mCurrentState.speed);
  }

  public Command changeState(State state){
    return new InstantCommand(() -> mCurrentState = state);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("IntakeSet", mCurrentState.speed);

    runIntake();
  }
}
