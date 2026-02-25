package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ArmStuff.Shooter;
import frc.robot.subsystems.ArmStuff.motor;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.commands.AprilTagCoordinates;


//While held, aim when in green zone, spin shooter to a looked-up target velocity based on distance, and run gumball when shooter is at speed. 
public class ShootButtonCommand extends Command {
    private final Swerve swerve;
    private final Shooter shooter;
    private final motor gumball;
    private final frc.robot.ShooterLookup lookup;
    private final PIDController pid;
    private final NetworkTableEntry txEntry = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx");
    private final NetworkTableEntry tvEntry = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv");

    private final double gumballSpeed = 0.2;

    public ShootButtonCommand(Swerve swerve, Shooter shooter, motor gumball, frc.robot.ShooterLookup lookup) {
        this.swerve = swerve;
        this.shooter = shooter;
        this.gumball = gumball;
        this.lookup = lookup;
        this.pid = new PIDController(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD);
        this.pid.setTolerance(1.0);
        addRequirements((Subsystem) swerve, (Subsystem) shooter, (Subsystem) gumball);
    }

    @Override
    public void initialize() {
        pid.reset();
    }

    @Override
    public void execute() {
        double dx = (swerve.getAprilOdom().getX() - 4.611624);
        double dy = (swerve.getAprilOdom().getY() - 4.021328);
        double distToGoal = Math.sqrt(dx * dx + dy * dy);
        boolean greenZone = (distToGoal <= 5) && (distToGoal >= 2);
        SmartDashboard.putBoolean("Green Zone", greenZone);
        SmartDashboard.putNumber("Green Zone Number", distToGoal);

        if (greenZone) {
            boolean hasTarget = tvEntry.getDouble(0.0) >= 1.0;
            if (hasTarget) {
                double tx = txEntry.getDouble(0.0);
        double TxGoal = Math.toDegrees(-1 * (Math.atan2(4.021328 - swerve.getAprilOdom().getY(), 4.611624 - swerve.getAprilOdom().getX()))
            + Math.atan2(AprilTagCoordinates.getY(26) - swerve.getAprilOdom().getY(), AprilTagCoordinates.getX(26) - swerve.getAprilOdom().getX()))
            + tx;
                double errorDeg = TxGoal - TurretConstants.CAMERA_TURRET_ANGLE_OFFSET_DEG;
                double rotCMD = pid.calculate(errorDeg, 0.0);
                rotCMD = Math.max(-TurretConstants.MAX_OUTPUT, Math.min(TurretConstants.MAX_OUTPUT, rotCMD));
                swerve.drive(new Translation2d(0.0, 0.0), rotCMD, true, true);
            }
        }

        // Get target RPM from lookup using measured distance
        double targetRpm = lookup.getSmoothedVelocity(distToGoal, 1.0);
        if (Double.isNaN(targetRpm)) {
            double manual = SmartDashboard.getNumber("Manual Shooter RPM", Double.NaN);
            if (!Double.isNaN(manual)) {
                targetRpm = manual;
            } else {
                targetRpm = 0.0;
            }
        }
        SmartDashboard.putNumber("Shooter Target RPM", targetRpm);

        shooter.setTargetRpm(targetRpm);

        double currentVel = shooter.getVelocity();
        SmartDashboard.putNumber("Shooter Current RPM", currentVel);
        boolean atSpeed = false;
        if (!Double.isNaN(currentVel)) {
            double tol = Math.max(25.0, Math.abs(targetRpm) * 0.02);
            atSpeed = Math.abs(currentVel - targetRpm) <= tol;
        }

        if (atSpeed) {
            gumball.setSpeed(gumballSpeed);
        } else {
            gumball.setSpeed(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setSpeed(0.0);
        gumball.setSpeed(0.0);
        pid.reset();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
