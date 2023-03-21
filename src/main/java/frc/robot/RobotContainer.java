// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.IntakeConstants.State;
import frc.robot.controls.Deadbander;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;

public class RobotContainer {

  private final Drivetrain mDrivetrain = new Drivetrain();
  private final Intake mIntake = new Intake();
  private final Pivot mPivot = new Pivot();
  private CommandXboxController mDriver = new CommandXboxController(0);
  private CommandXboxController mOperator = new CommandXboxController(1);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // mDrivetrain.setDefaultCommand(
    //   new RunCommand(()-> mDrivetrain.drive(
    //     -Deadbander.applyLinearScaledDeadband(mDriver.getLeftY(), 0.1),
    //     -Deadbander.applyLinearScaledDeadband(mDriver.getRightX(), 0.1),
    //      mDriver.rightBumper().getAsBoolean()     
    //   ),
    //   mDrivetrain)
    // );

    mDriver.a().onTrue(
      mIntake.changeState(IntakeConstants.State.GRAB)
    );
    mDriver.a().onFalse(
      mIntake.changeState(IntakeConstants.State.IDLE)
    );

    mDriver.b().onTrue(
      mIntake.changeState(IntakeConstants.State.RELEASE)
    );
    mDriver.b().onFalse(
      mIntake.changeState(IntakeConstants.State.IDLE)
    );

    mDriver.povRight().onTrue(
      new InstantCommand(
        mPivot::zeroEncoder,
        mPivot
      )
    );

    mDriver.x().onTrue(
      mPivot.changeState(PivotConstants.State.L2)
    );

    mDriver.y().onTrue(
      mPivot.changeState(PivotConstants.State.CARRY)
    );

    mDriver.rightBumper().onTrue(
      mPivot.changeState(PivotConstants.State.L1)
    );

    mDriver.leftBumper().onTrue(
      mPivot.changeState(PivotConstants.State.CARRY)
    );

  

    mOperator.a().onTrue(
      new ParallelCommandGroup(
        mIntake.changeState(IntakeConstants.State.GRAB),
        mPivot.changeState(PivotConstants.State.SUBSTATION)
      )
    );

    mOperator.a().onFalse(
      new ParallelCommandGroup(
        new WaitCommand(0.5),
        mIntake.changeState(IntakeConstants.State.IDLE),
        mPivot.changeState(PivotConstants.State.CARRY)
      )
    );

    mOperator.b().onTrue(
      new SequentialCommandGroup(
        mPivot.changeState(PivotConstants.State.L2),
        new WaitCommand(0.25),
        mIntake.changeState(IntakeConstants.State.RELEASE)
      )
    );
    
    mOperator.b().onFalse(
      new ParallelCommandGroup(
        mIntake.changeState(IntakeConstants.State.IDLE),
        mPivot.changeState(PivotConstants.State.CARRY)
      )
    );

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
