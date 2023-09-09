package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

public class SwerveModule {
    
    public CANSparkMax drive;
    public CANSparkMax spin;
    public AbsoluteEncoder turningEncoder;

    public SwerveModule(int driveNumber, int spinNumber)
    {
        drive = new CANSparkMax(driveNumber, MotorType.kBrushless);
        spin = new CANSparkMax(spinNumber, MotorType.kBrushless);

        turningEncoder = spin.getAbsoluteEncoder(Type.kDutyCycle);
    }

    public void Drive(double speed) {
        System.out.println(turningEncoder.getPosition());

        drive.set(speed);
    }
      
    public void spin(double speed) {
        spin.set(speed);
    }
}