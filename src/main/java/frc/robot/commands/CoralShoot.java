package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmStuff.Shooter;
import frc.robot.subsystems.swerve.Swerve;

public class CoralShoot extends Command {
    private Swerve swerve;
    private Shooter shooter;
    private BooleanSupplier shooter1;
    private BooleanSupplier shooter2;
    

    public CoralShoot(Swerve s_Swerve, Shooter shooter, BooleanSupplier shooter1, BooleanSupplier shooter2) {
        this.swerve = s_Swerve;
        this.shooter = shooter;
        addRequirements(shooter);
        this.shooter1 = shooter1;
        this.shooter2 = shooter2;
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        boolean GreenZone = (Math.sqrt(Math.pow((swerve.getAprilOdom().getX() - 4.611624), 2) + Math.pow((swerve.getAprilOdom().getY() - 4.021328), 2)) <= 5) && (Math.sqrt(Math.pow((swerve.getAprilOdom().getX() - 4.611624), 2) + Math.pow((swerve.getAprilOdom().getY() - 4.021328), 2)) >= 2);
        SmartDashboard.putBoolean("Green Zone", GreenZone);
        SmartDashboard.putNumber("Green Zone Number", Math.sqrt(Math.pow((swerve.getAprilOdom().getX() - 4.611624), 2) + Math.pow((swerve.getAprilOdom().getY() - 4.021328), 2)));
        SmartDashboard.putNumber("Shooter get Velocity", shooter.getVelocity());
        if (!GreenZone) {
            if (shooter1.getAsBoolean() == true){
                shooter.setSpeed(0.2);
            }
            else {
                shooter.setSpeed(0);
            }  
        }
        else {
            shooter.setSpeed(0);
        }

    }
    @Override
    public boolean isFinished() {
        return false;
    }
    
    @Override
	public void end(boolean interrupted){
		
	}    
}