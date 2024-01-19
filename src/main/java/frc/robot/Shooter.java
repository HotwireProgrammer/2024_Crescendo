package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class Shooter {
    public float sppeed = 0;

    public TalonSRX shooterTwo = new TalonSRX(6);
    public TalonSRX shooterFive = new TalonSRX(50);

    public Limelight limelight;
    public PreShooterpid preshooterpid;
    public double shooterP = 0.0002;
    public double shooterI = 0.0004;
    public double shooterD = 0.0;

    public HotPID pid;

    public boolean didHitSpeed = true;

    public double rpmTarget = 0;

    public float rpmCurrent = 0;

    public Shooter(Limelight limelight, PreShooterpid preshooterpid) {

        this.limelight = limelight;
        this.preshooterpid = preshooterpid;
    }

    public void Init() {
        SmartDashboard.putNumber("Shooter Rot Target", sppeed);
        SmartDashboard.putNumber("ShooterP", shooterP);
        SmartDashboard.putNumber("ShooterI", shooterI);
        SmartDashboard.putNumber("ShooterD", shooterD);
        shooterP = SmartDashboard.getNumber("ShooterP", shooterP);
        shooterI = SmartDashboard.getNumber("ShooterI", shooterI);
        shooterD = SmartDashboard.getNumber("ShooterD", shooterD);

        pid = new HotPID("shooter", shooterP, shooterI, shooterD);
    }

    public void Reset() {
        sppeed = (float) SmartDashboard.getNumber("Shooter Rot Target", sppeed);

        pid.reset();
    }

    public void Update() {

        float nearY = 34.8f;// (34.8, 1600) 2221*0.982^dist
        // 2 ft spacing (28.3, 1700)
        // 2 ft spacing (23.6, 1850)
        // 2 ft spacing (19.7, 2000)
        // 2 ft spacing (17, 2150)
        // 2 ft spacing (15 , 2300)
        float farY = 15f;// 20.7

        float dist = Math.abs(farY - nearY);
        float distT = (float) limelight.GetY() - 15.0f;
        float distCurve = (1.546f * (distT * distT)) - (65.06f * distT) + 2286.0f + 150.0f;
        float relDist = distCurve;

        // shooter

        float nearShooterRPM = 1600;// 1600//1400
        float farShooterRPM = 2300;// hangar/p/2100//2050

        // float shooterSpeed = Lerp(nearShooterRPM, farShooterRPM, relDist);

        if ((limelight.GetArea() == 0)) {
            rpmTarget = 1900;
        } else {
            if (limelight.GetY() < 15) {
                rpmTarget = 2300;
            } else {
                rpmTarget = relDist;
            }
        }
        // preshooter
        float nearPreRPM = 1700;// 1700
        float farPreRPM = 1800;// 1817
        float preShooterSpeed = Lerp(nearPreRPM, farPreRPM, relDist);
        preshooterpid.preRpmTarget = preShooterSpeed;

        preshooterpid.Update();

        rpmCurrent = TalonVelocityToRPM((float) shooterTwo.getSelectedSensorVelocity());

        // Shooter code pid
        SmartDashboard.putNumber("Shooter_RPM", rpmCurrent);

        pid.setpoint = rpmTarget;
        double motorSpeed = pid.Calculate(rpmCurrent);

        if (motorSpeed < 0) {
            motorSpeed = 0;
        }
        float max = 0.95f;
        if (motorSpeed >= max) {
            motorSpeed = max;
        }
        // System.out.println("Speed " + motorSpeed + " RPM " + rpm);
        SmartDashboard.putNumber("Shooter_Speed", motorSpeed);
        if (rpmTarget == 0.0) {
            PowerManual(0);
        } else {
            PowerManual((float) motorSpeed);
        }
    }

    public boolean UpToSpeed(float RPMBuffer) {
        double distance = Math.abs(rpmCurrent - rpmTarget);
        return distance < (rpmTarget * RPMBuffer);
    }

    public void PowerManual(float power) {
        shooterFive.set(ControlMode.PercentOutput, power);
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
