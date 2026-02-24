package frc.robot.subsystems.ArmStuff;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.ArmConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;



public class motor extends SubsystemBase implements ArmConstants {
    public SparkMax motor1;
    public RelativeEncoder motor1Encoder;

    public motor() {
        motor1 = new SparkMax(motorid, MotorType.kBrushless);
        motor1Encoder = motor1.getEncoder();
        motor1Encoder.setPosition(0);
    }
    public void setSpeed(double s) {
        motor1.set(s);
    }

    public double getVelocity() {
        return motor1Encoder.getVelocity();
    }

    public double getIntakeCurrent() {
        return motor1.getOutputCurrent();
    }
    public double getMotorPos(){
        return motor1Encoder.getPosition();
    }


}
