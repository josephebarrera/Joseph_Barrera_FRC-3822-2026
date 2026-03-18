package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.swervedrive.AgitatorSubsystem;
import frc.robot.subsystems.swervedrive.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;

import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

    final CommandXboxController driverXbox = new CommandXboxController(0);
    // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));

    //Created a shooter
    ShooterSubsystem shooter = new ShooterSubsystem();
    //Created a agitator
    AgitatorSubsystem agitator = new AgitatorSubsystem();
    //Create a intake
    IntakeSubsystem intake = new IntakeSubsystem();

    /**
    * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
    */
    public SwerveInputStream driveInputStream = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * 1,
      () -> driverXbox.getLeftX() * 1)
      .withControllerHeadingAxis(() -> driverXbox.getRightX() * 1, () -> driverXbox.getRightY() * 1)
      .deadband(Constants.OperatorConstants.DEADBAND)
      .scaleTranslation(0.5)
      .allianceRelativeControl(true)
      .headingWhile(true);

    /**
    * The container for the robot. Contains subsystems, OI devices, and commands.
    */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    setupPathPlannerCommands();
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private void setupPathPlannerCommands()
  {
    NamedCommands.registerCommand("Shoot Forward", shooter.shootForward());
  }

  private void configureBindings()
  {
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveInputStream);
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    driverXbox.start().onTrue(
      Commands.runOnce(() ->
      {
        drivebase.resetOdometry(null);
      }));

      //Right trigger
      driverXbox.rightTrigger()
        .whileTrue(shooter.shootForward())
        .onFalse(shooter.shootStop());

      //driverXbox.leftBumper()
          //.toggleOnTrue();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // Pass in the selected auto from the SmartDashboard as our desired autnomous commmand 
    return null;
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
