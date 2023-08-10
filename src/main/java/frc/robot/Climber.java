package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jdk.jfr.Percentage;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Climber {

    public Boolean solenoidToggle = false;
    public TalonSRX climberOne = new TalonSRX(35);
    public TalonSRX climberTwo = new TalonSRX(52);
    public DoubleSolenoid lockDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);
    public double encodererror1;
    public double encodererror2;

    public float changeDelay = 0.2f;

    public void errorSet() {
        encodererror1 = climberOne.getSelectedSensorPosition();
        encodererror2 = climberTwo.getSelectedSensorPosition();
    }

    public void coastMode() {
        climberOne.setNeutralMode(NeutralMode.Coast);
        climberTwo.setNeutralMode(NeutralMode.Coast);
    }

    public void brakeMode() {
        climberOne.setNeutralMode(NeutralMode.Brake);
        climberTwo.setNeutralMode(NeutralMode.Brake);
    }

    public void Tilt() {
        if (solenoidToggle) {
            solenoidToggle = false;
            lockDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
        } else {
            solenoidToggle = true;
            lockDoubleSolenoid.set(DoubleSolenoid.Value.kForward);
        }
    }

    public void climbMotors(float climbSpeed) {
        if (climbSpeed > 0) {
            if ((climberTwo.getSelectedSensorPosition() - encodererror2) > -183835) {// -179000
                climberTwo.set(ControlMode.PercentOutput, -climbSpeed);
            } else {
                System.out.println("hello");
                climberTwo.set(ControlMode.PercentOutput, 0);
            }
            if ((climberOne.getSelectedSensorPosition() - encodererror1) < 183835) {// 183000
                climberOne.set(ControlMode.PercentOutput, climbSpeed);
            } else {
                climberOne.set(ControlMode.PercentOutput, 0);
            }
        } else {
            climberOne.set(ControlMode.PercentOutput, climbSpeed);
            climberTwo.set(ControlMode.PercentOutput, -climbSpeed);
        }
        brakeMode();

    }
}
