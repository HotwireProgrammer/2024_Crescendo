package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class SwerveModule {
    
    public TalonSRX drive;
    public TalonSRX spin;

    public SwerveModule(int driveNumber, int spinNumber)
    {
        drive = new TalonSRX(driveNumber);
        spin = new TalonSRX(spinNumber);
    }

    public void Drive(double speed) {
        drive.set(TalonSRXControlMode.PercentOutput, speed);
    }
      
    public void spin(double speed) {
        spin.set(TalonSRXControlMode.PercentOutput, speed);
    }
}