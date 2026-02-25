package frc.robot.subsystems.ArmStuff;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import frc.robot.ArmConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;


public class Shooter extends SubsystemBase implements ArmConstants  {

    public SparkMax coralShooter1;
    public RelativeEncoder shooterEncoder;
    private SparkClosedLoopController closedLoop;
   // public SparkMax coralShooter2;
   
    

    public Shooter(){
        coralShooter1 = new SparkMax(shooterId1, MotorType.kBrushless);
        shooterEncoder = coralShooter1.getEncoder();
        
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop
            .p(0.00020)
            .i(0.0)
            .d(0.0)
            .outputRange(-1.0, 1.0);
        config.closedLoop.feedForward
            .kV(0.00013);
        coralShooter1.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        closedLoop = coralShooter1.getClosedLoopController();
        //coralShooter2 = new SparkMax(shooterId2, MotorType.kBrushless);
    }

    public void setSpeed(double s){
        coralShooter1.set(s);
       // coralShooter2.set(s * -1);
    }

    // Set shooter target in RPM using closed-loop controller
    public void setTargetRpm(double rpm) {
        if (closedLoop != null) {
            closedLoop.setReference(rpm, ControlType.kVelocity);
        } else {
            coralShooter1.set(rpm/5000.0);
        }
    }

    public void stopShooter() {
        coralShooter1.stopMotor();
    }

    public double getVelocity() {
        return shooterEncoder.getVelocity();
    }

    public void setVelocity(double speed) {
        coralShooter1.set(speed);
    }

    public double getIntakeCurrent() {
        return coralShooter1.getOutputCurrent();
    }

    
}
