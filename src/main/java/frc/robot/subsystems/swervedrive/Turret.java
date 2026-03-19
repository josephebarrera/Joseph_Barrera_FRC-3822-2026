package frc.robot.subsystems.swervedrive;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase
{
    //Use Sparkmax to create the motors for the Turret
    SparkMax turret = new SparkMax(11, MotorType.kBrushless);

    private final PIDController aimPID = new PIDController(0.02, 0.0, 0.0);

     public Turret()
    {
        aimPID.setTolerance(1.0);
    }

    public void setTurretPower(double power)
    {
        turret.set(MathUtil.clamp(power, -0.35, 0.35));
    }

    public void stopTurret()
    {
        turret.set(0.0);
    }

    public void aimAtTarget(double yawErrorDegrees)
    {
        double output = aimPID.calculate(yawErrorDegrees, 0.0);
        setTurretPower(output);
    }

    public boolean aimedAtTarget()
    {
        return aimPID.atSetpoint();
    }

}