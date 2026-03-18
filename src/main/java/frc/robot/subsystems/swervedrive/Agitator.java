package frc.robot.subsystems.swervedrive;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AgitatorSubsystem extends SubsystemBase
{
    //Use SparkMax to create the motors for the agitator
    SparkMax agitator = new SparkMax(15, MotorType.kBrushless);

    //************************************************* Commands *************************************************/
    public Command funnelForward()
    {
        return Commands.runOnce(()->
        {
            System.out.println("Setting funnel speed to ");
            agitator.set(-1);
        });
    }

    public Command funnelReverse()
    {
        return Commands.runOnce(()->
        {
            agitator.set(-100.0);
        });
    }

    public Command funnelStop() 
    {
        return Commands.runOnce(()->
        {
            agitator.set(0.0);
        });
    }
}