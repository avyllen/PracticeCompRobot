package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkMax;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.generated.Constants.LimeLightConstants;
import frc.robot.generated.Constants.PivotConstants;
import frc.robot.generated.Constants.ShooterConstants;;

public class PivotSubsystem extends SubsystemBase{

private CANSparkMax m_pviot;
private SparkPIDController m_pidController;
private RelativeEncoder m_encoder;
public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;


public PivotSubsystem()
{
  m_pviot = new CANSparkMax(PivotConstants.pivot, MotorType.kBrushless);
  m_pviot.restoreFactoryDefaults();
  /**
     * In order to use PID functionality for a controller, a SparkPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
  m_pidController = m_pviot.getPIDController();

   // Encoder object created to display position values
   m_encoder = m_pviot.getEncoder();

   // PID coefficients
   kP = 0.2; 
   kI = 1e-4;
   kD = 1; 
   kIz = 0; 
   kFF = 0; 
   kMaxOutput = 1; 
   kMinOutput = -1;

   // set PID coefficients
   m_pidController.setP(kP);
   m_pidController.setI(kI);
   m_pidController.setD(kD);
   m_pidController.setIZone(kIz);
   m_pidController.setFF(kFF);
   m_pidController.setOutputRange(kMinOutput, kMaxOutput);

   m_encoder.setPosition(0);
}

public void setVelocity(double setPoint)
{
  //m_pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
m_pviot.set(setPoint);
}

public void setPosition(double setPoint)
{
  m_pidController.setReference(setPoint, CANSparkMax.ControlType.kPosition);
}

public void intakePosition()
{
  m_pidController.setReference(9.95, CANSparkMax.ControlType.kPosition);
}

public void subwooferPosition()
{
  m_pidController.setReference(12.5, CANSparkMax.ControlType.kPosition);
}

public void otherPositions()
{
  m_pidController.setReference(12.5, CANSparkMax.ControlType.kPosition);
}

public Command withPosition(double setPoint)
{
  return run(() -> this.setPosition(setPoint));
}



public Command slowUp()
{
  return run(() -> this.setVelocity(.1));
}

public Command slowDown()
{
  return run(() -> this.setVelocity(-.1));
}

public Command stop()
{
  return run(() -> this.setVelocity(0));
}

public Command holdPosition()
{
  return run(() -> this.setPosition(this.m_encoder.getPosition()));
}

public boolean LimitChecks()
{
return ((m_encoder.getPosition() < 0.7 && m_pviot.getAppliedOutput() < 0) || (m_encoder.getPosition() > 37 && m_pviot.getAppliedOutput() > 0));
}

//Limelight
public double getDistance(double ty)
{
    return ((LimeLightConstants.goalHeightInches - LimeLightConstants.limelightLensHeightInches) / 
    Math.tan(Math.toRadians(LimeLightConstants.limelightMountAngledegrees) + Math.toRadians(ty)));

}



// lightlightAutoAim

public Command lightlightAutoAim (double ty) {

  return run(() -> this.setPosition(MathUtil.clamp((((this.getDistance(LimelightHelpers.getTY("limelight-backup")) - 36.125) * 0.1194) + 25.66), 0.0, 37)));
}

public Command advancedLightlightAutoAimV1 (double ty, double yVelocity) {

  double offset = (yVelocity * 0.7);
  return run(() -> this.setPosition(MathUtil.clamp((((offset + this.getDistance(LimelightHelpers.getTY("limelight-backup")) - 36.125) * 0.1194) + 25.66), 0.0, 37)));
}



@Override
public void periodic() {
  // This method will be called once per scheduler run
SmartDashboard.putNumber("Shooter Pivot Encoder", m_encoder.getPosition());
SmartDashboard.putNumber("Shooter Pivot Appied Voltage", m_pviot.getAppliedOutput());

SmartDashboard.putNumber("distance", this.getDistance(LimelightHelpers.getTY("limelight-backup")));
SmartDashboard.putNumber("calculated encoder value", (MathUtil.clamp((((this.getDistance(LimelightHelpers.getTY("limelight-backup")) - 36.125) * 0.1194) + 25.66), 0.0, 37))); //24.96
}


}
