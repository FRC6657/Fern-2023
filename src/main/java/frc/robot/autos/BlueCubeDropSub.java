package frc.robot.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;

public class BlueCubeDropSub extends SequentialCommandGroup {

  private final Drivetrain mDrivetrain;
  private final Intake mIntake;
  private final Pivot mPivot;
  
  /** Creates a new CubeDropTaxi. */
  public BlueCubeDropSub(Drivetrain _drivetrain, Intake _intake, Pivot _pivot) {
    mDrivetrain = _drivetrain;
    mIntake = _intake;
    mPivot = _pivot;

    addRequirements(mDrivetrain, mIntake, mPivot);

    addCommands(
        new InstantCommand(mDrivetrain::resetGyro),
      mPivot.changeState(PivotConstants.State.L1),
      new WaitCommand(0.5),
      mIntake.changeState(IntakeConstants.State.L1RELEASE),
      new WaitCommand(1),
      mPivot.changeState(PivotConstants.State.CARRY),
      mIntake.changeState(IntakeConstants.State.STOP),
      new WaitCommand(0.25),
      mDrivetrain.new RotateRelative(Rotation2d.fromDegrees(90)),
      new RunCommand(()-> mDrivetrain.drive(0.5, 0, false)).withTimeout(1)
    );

  }
}
