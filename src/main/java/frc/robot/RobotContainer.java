// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.IntakeConstants.State;
import frc.robot.autos.RedCubeDropTaxi;
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

  private SendableChooser<SequentialCommandGroup[]> mAutoChooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

    mDrivetrain.setDefaultCommand(
      new RunCommand(()-> mDrivetrain.drive(
        -Deadbander.applyLinearScaledDeadband(mDriver.getLeftY(), 0.1)* (mDriver.leftTrigger().getAsBoolean() ? Constants.DriveConstants.kTurboForwardSpeed : Constants.DriveConstants.kNormalForwardSpeed),
        Deadbander.applyLinearScaledDeadband(mDriver.getRightX(), 0.1)* (mDriver.leftTrigger().getAsBoolean() ? Constants.DriveConstants.kTurboTurningSpeed : Constants.DriveConstants.kNormalTurningSpeed),
         mDriver.rightBumper().getAsBoolean()     
      ),
      mDrivetrain)
    );


    mDriver.povUp().whileTrue(
      new InstantCommand(
        () -> mDrivetrain.forward(0.1),
        mDrivetrain
      )
    );

    mDriver.povUp().onFalse(
      new InstantCommand(
        () -> mDrivetrain.forward(0),
        mDrivetrain
      )
    );

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

    mDriver.povUp().onTrue(
      mDrivetrain.changeState(DriveConstants.State.FORWARD)
    );

    mDriver.povDown().onTrue(
      mDrivetrain.changeState(DriveConstants.State.REVERSE)
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


    mDriver.y().onTrue(
      new SequentialCommandGroup(
        mPivot.changeState(PivotConstants.State.FLOOR),
        new WaitCommand(0.25),
        mIntake.changeState(IntakeConstants.State.GRAB)
      )
    );
    
    mDriver.y().onFalse(
      new ParallelCommandGroup(
        mIntake.changeState(IntakeConstants.State.IDLE),
        mPivot.changeState(PivotConstants.State.CARRY)
      )
    );

  

    mOperator.a().onTrue(
      mIntake.changeState(IntakeConstants.State.RELEASE)
    );

    mOperator.a().onFalse(
      new SequentialCommandGroup(
        mIntake.changeState(IntakeConstants.State.STOP),
        mPivot.changeState(PivotConstants.State.CARRY)
      )
    );

    mOperator.b().onTrue(
      new SequentialCommandGroup(
        mPivot.changeState(PivotConstants.State.SUBSTATION),
        mIntake.changeState(IntakeConstants.State.GRAB)
      )
    );

    mOperator.b().onFalse(
      new SequentialCommandGroup(
        mIntake.changeState(IntakeConstants.State.STOP),
        mPivot.changeState(PivotConstants.State.CARRY)
      )
    );

    mOperator.y().onTrue(
      new SequentialCommandGroup(
        mPivot.changeState(PivotConstants.State.L2),
        new WaitCommand(0.07),
        mIntake.changeState(IntakeConstants.State.RELEASE)
      )
    );

    mOperator.y().onFalse(
      new SequentialCommandGroup(
        mIntake.changeState(IntakeConstants.State.STOP),
        mPivot.changeState(PivotConstants.State.CARRY)
      )
    );





    mOperator.povUp().onTrue(
      mPivot.changeState(PivotConstants.State.L2)
    );

    mOperator.povDown().onTrue(
      mPivot.changeState(PivotConstants.State.L1)
    );


    configureAutoChooser();

  }

public void configureAutoChooser() {
  mAutoChooser.setDefaultOption("Nothing", new SequentialCommandGroup[]{null, null});

  mAutoChooser.addOption("Cube L1 Taxi", new SequentialCommandGroup[]{
    new RedCubeDropTaxi(mDrivetrain, mIntake, mPivot),
    new RedCubeDropTaxi(mDrivetrain, mIntake, mPivot)
  });

  SmartDashboard.putData(mAutoChooser);

}

public SequentialCommandGroup getAutonomousCommand() {
        
        int alliance = 0;
        if(DriverStation.getAlliance() == Alliance.Blue){
          alliance = 0;
        }else{
          alliance = 1;
        }

         return mAutoChooser.getSelected()[alliance];
      }
    }
