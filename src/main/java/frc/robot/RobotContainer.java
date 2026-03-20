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
import frc.robot.subsystems.swervedrive.Turret;
import java.io.File;
import com.pathplanner.lib.auto.NamedCommands;
import swervelib.SwerveInputStream;
import frc.robot.subsystems.swervedrive.Vision;
import frc.robot.commands.swervedrive.AimTurretCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  final CommandXboxController driverXbox = new CommandXboxController(0);
   final CommandXboxController shooterXbox = new CommandXboxController(1);

    //The robot's subsystems and commands are defined here...
    private final SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));

    //If anything goes wrong delete line 36
    private final Vision vision = new Vision(drivebase::getPose, drivebase.getSwerveDrive().field);

    //Created a shooter
    Shooter shooter = new Shooter();

    //Created a agitator
    Agitator agitator = new Agitator();

    //Create a intake
    Intake intake = new Intake();

    //Create a turret
    Turret turret = new Turret();

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
      
  //This is robot oriented driving. Don't touch it.
    public SwerveInputStream driveDirectAngle = driveInputStream.copy()
                        .withControllerHeadingAxis(() -> driverXbox.getRightX()*-1,
                                        () -> driverXbox.getRightY()*-1)
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
      NamedCommands.registerCommand("Shoot Forward", shooter.shootForward());
      NamedCommands.registerCommand("Open Intake Middle", intake.foldOpenIntake());
    }

    private void configureBindings()
    {
      driverXbox.b().onTrue(Commands.runOnce(drivebase::zeroGyro));

      /****************************************************** Trial ******************************************************/

      //Enable Intake
      shooterXbox.leftBumper()
      .whileTrue(intake.spinIntakeForward())
      .onFalse(intake.stopIntake());

      //Open intake
      shooterXbox.povDown()
      .whileTrue((intake.foldOpenIntake()));

      //Close intake
      shooterXbox.povUp()
      .whileTrue(intake.foldCloseIntake());

      shooterXbox.rightBumper()
      .toggleOnTrue(shooter.toggleTopShooter());

      //Shoot
      shooterXbox.rightTrigger()
        .whileTrue(funnelAndShoot())
        .onFalse(Commands.parallel(agitator.funnelStop(),shooter.shootStop()));

      driverXbox.a()
        .whileTrue(Commands.run(() -> turret.testTurnRight(), turret))
        .onFalse(Commands.runOnce(() -> turret.stopTurret(), turret));

      driverXbox.b()
        .whileTrue(Commands.run(() -> turret.testTurnLeft(), turret))
        .onFalse(Commands.runOnce(() -> turret.stopTurret(), turret));

      //shooterXbox.povLeft().whileTrue(turret.testTurnLeft());
      //shooterXbox.povRight().whileTrue(turret.testTurnRight());

      // //Reset odometry
      //  driverXbox.start().onTrue(
      //   Commands.runOnce(() ->
      //   {
      //     drivebase.resetOdometry(null);
      //   }));

      /******************************************************************************************************************/

      Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveInputStream);
      Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

       
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
        
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
      //Return the swerve subsystem command for this path
      return null;
    }

    public void setMotorBrake(boolean brake)
    {
      drivebase.setMotorBrake(brake);
    }

     /*********************************************************************** Commands *************************************************************************/
    public Command funnelAndShoot()
    {
      return Commands.parallel(
      agitator.funnelForward(),
      shooter.spinShooterIntake());
    }
}
