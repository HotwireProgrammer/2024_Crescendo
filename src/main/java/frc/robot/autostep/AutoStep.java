package frc.robot.autostep;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.DriveTrain;

public abstract class AutoStep {

    public boolean isDone;

    public boolean autoIndex = true;
    public boolean runShooter = false;

    public AutoStep()
    {
        isDone = false;
    }

    public abstract void Begin();
    public abstract void Update();
}