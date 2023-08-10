package frc.robot.autostep;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Timer;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.DriveTrain;

public class IntakeRun extends AutoStep {

    public TalonSRX motor; 
    public double speed;

    public IntakeRun(TalonSRX motor, float speed) {
        super();
        this.motor = motor;
        this.speed = speed;
    }

    public void Begin() {
        motor.set(ControlMode.PercentOutput, speed);
    }

    public void Update() {
        isDone = true;
    }
}