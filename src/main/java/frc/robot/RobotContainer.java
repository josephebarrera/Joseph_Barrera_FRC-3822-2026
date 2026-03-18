package frc.robot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.swervedrive.Agitator;
import frc.robot.subsystems.swervedrive.Intake;
import frc.robot.subsystems.swervedrive.Shooter;
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
    Shooter shooter = new Shooter();
    //Created a agitator
    Agitator agitator = new Agitator();
    //Create a intake
    Intake intake = new Intake();

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
      //Configure the trigger bindings
      configureBindings();

      //Configure the PathPlanner commands
      setupPathPlannerCommands();

      DriverStation.silenceJoystickConnectionWarning(true);
    }

    private void setupPathPlannerCommands()
    {
      //Register Commands:
      NamedCommands.registerCommand("Shoot Forward", shooter.shootForward());
      NamedCommands.registerCommand("Open Intake Middle", intake.foldOpenIntake());
    }

    private void configureBindings()
    {
      Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveInputStream);
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

      //Reset Odomometry on the Xbox Controller
      driverXbox.start().onTrue(
        Commands.runOnce(() ->
        {
          drivebase.resetOdometry(null);
        }));

      //Right trigger on the Xbox Controller
      driverXbox.rightTrigger()
        .whileTrue(shooter.shootForward())
        .onFalse(shooter.shootStop());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
          // Return the swerve subsystem command for this path
    return null;
    }

    public void setMotorBrake(boolean brake)
    {
      drivebase.setMotorBrake(brake);
    }

     /*********************************************************************** Commands *************************************************************************/
    public Command intakeBalls()
    {
      return Commands.parallel(
                  
                  intake.spinIntakeForward(),
                  agitator.funnelForward()
            );
    }

}
