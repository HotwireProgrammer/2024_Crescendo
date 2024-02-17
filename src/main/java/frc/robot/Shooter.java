package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class Shooter {
    public float sppeed = 0;

    public TalonSRX shooterOne = new TalonSRX(6);
    public TalonSRX shooterTwo = new TalonSRX(50);

    public Limelight limelight;

    public double shooterP = 0.0002;
    public double shooterI = 0.0004;
    public double shooterD = 0.0;

    public HotPID shooterPid;

    public boolean didHitSpeed = true;

    public double rpmTarget = 0;

    public float rpmCurrent = 0;

    public Shooter(Limelight limelight, PreShooterpid preshooterpid) {

        this.limelight = limelight;
    }

    public void Init() {
        shooterPid = new HotPID("shooter", shooterP, shooterI, shooterD);
    }

    public void Reset() {
        sppeed = (float) SmartDashboard.getNumber("Shooter Rot Target", sppeed);

        shooterPid.reset();
    }

    public void Update() {

        rpmCurrent = TalonVelocityToRPM((float) shooterTwo.getSelectedSensorVelocity());
        rpmTarget = 1000;

        shooterPid.setpoint = rpmTarget;
        double motorSpeed = shooterPid.Calculate(rpmCurrent);

        if (motorSpeed < 0) {
            motorSpeed = 0;
        }

        float max = 0.95f;
        if (motorSpeed >= max) {
            motorSpeed = max;
        }

        if (rpmTarget == 0.0) {
            PowerManual(0);
        } else {
            PowerManual((float) motorSpeed);
        }

        SmartDashboard.putNumber("Shooter_Speed", motorSpeed);
        SmartDashboard.putNumber("Shooter_RPM", rpmCurrent);
    }

    public boolean UpToSpeed(float RPMBuffer) {
        double distance = Math.abs(rpmCurrent - rpmTarget);
        return distance < (rpmTarget * RPMBuffer);
    }

    public void PowerManual(float power) {
        shooterOne.set(ControlMode.PercentOutput, power);
        shooterTwo.set(ControlMode.PercentOutput, power * -1);
    }

    public float TalonVelocityToRPM(float ticks) {
        float rpm = ((ticks / 2048) * 600);
        return Math.abs(rpm);
    }

    public float Lerp(float v0, float v1, float t) {

        if (t <= 0) {
            t = 0;
        } else if (t >= 1) {
            t = 1;
        }

        return (v0 + t * (v1 - v0));
    }
}
