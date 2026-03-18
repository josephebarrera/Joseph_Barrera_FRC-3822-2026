package frc.robot.subsystems.swervedrive;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase
{
    //Use the SparkMax to create the motors for the Intake
    SparkMax intake = new SparkMax(10, MotorType.kBrushless);

    //************************************************* Commands *************************************************/
    public Command spinIntakeForward() 
    {
        return Commands.runOnce(()->
        {
            intake.set(-100.0);
        });
    }

    public Command spinIntakeReverse() 
    {
        return Commands.runOnce(()->
        {
            intake.set(100.0);
        });
    }

    public Command spinIntakeStop() 
    {
        return Commands.runOnce(()->
        {
            intake.set(0.0);
        });
    }
}