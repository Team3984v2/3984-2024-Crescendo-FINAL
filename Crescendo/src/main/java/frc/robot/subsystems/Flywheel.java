package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.flywheel;

public class Flywheel extends SubsystemBase{
    private CANSparkMax top;
    private CANSparkMax bott;
    private RelativeEncoder topEncoder;
    private RelativeEncoder bottEncoder;
    private SparkPIDController FWtopPID;
    private SparkPIDController FWbottPID;
    public Flywheel() {
        // initialies all the variables and constants 
        top = new CANSparkMax(frc.robot.Constants.Swerve.flywheel.FWtop.FWid, MotorType.kBrushless );
        bott = new CANSparkMax(frc.robot.Constants.Swerve.flywheel.FWbott.FWid, MotorType.kBrushless );
        top.restoreFactoryDefaults();
        bott.restoreFactoryDefaults();
        top.setIdleMode(IdleMode.kCoast);
        bott.setIdleMode(IdleMode.kCoast);
        bott.setInverted(true);
        top.setInverted(false);
        topEncoder = top.getEncoder(
            SparkRelativeEncoder.Type.kHallSensor, 
            42
        );
        topEncoder.setVelocityConversionFactor(
            (Math.PI * Constants.Swerve.flywheel.FWDiameter * 0.0254 / Constants.Swerve.flywheel.FWtop.gearRatio) / 60.0 
        );
        topEncoder.setPosition(0);
        bottEncoder = bott.getEncoder(
            SparkRelativeEncoder.Type.kHallSensor, 
            42
        );
        bottEncoder.setVelocityConversionFactor(
            (Math.PI * Constants.Swerve.flywheel.FWDiameter * 0.0254 / Constants.Swerve.flywheel.FWbott.gearRatio) / 60.0
        );
        bottEncoder.setPosition(0);
        
        FWtopPID = top.getPIDController();
        FWtopPID.setP(Constants.Swerve.flywheel.FWtop.kP);
        FWtopPID.setI(Constants.Swerve.flywheel.FWtop.kI);
        FWtopPID.setD(Constants.Swerve.flywheel.FWtop.kD);

        FWbottPID = bott.getPIDController();
        FWbottPID.setP(Constants.Swerve.flywheel.FWbott.kP);
        FWbottPID.setI(Constants.Swerve.flywheel.FWbott.kI);
        FWbottPID.setD(Constants.Swerve.flywheel.FWbott.kD);
        top.burnFlash();
        bott.burnFlash();
    }

    public double[] getVelocity(){
        double topVelocity = topEncoder.getVelocity(); //RPM
        double bottVelocity = bottEncoder.getVelocity();
        double[] combinedVelocity = 
        new double[] {topVelocity , bottVelocity};
        return combinedVelocity;
    }
    public double[] getErrors(double[] goal){
        double[] currVelocity = new double[] {getVelocity()[0], getVelocity()[1]};
        double[] error = new double[] {
            goal[0] - currVelocity[0] , goal[1] - currVelocity[1]
        };
        return error;
    }
    public void GoTo(double topGoal, double bottGoal){
        FWtopPID.setReference(
            topGoal, 
            ControlType.kVelocity, 0, new SimpleMotorFeedforward(0, 0.1).calculate(topGoal)
        );
        FWbottPID.setReference(
            bottGoal, 
            ControlType.kVelocity, 0, new SimpleMotorFeedforward(0, 0.1).calculate(bottGoal)
        );
    }
    private boolean atSetpoint = false;

    public Command stop(){
        return run(()->{top.set(0); bott.set(0);});
    }
    public Command ampShot(){
        return run(()->{top.setVoltage(1.70); bott.setVoltage(1.70);});
    }

    public Command reverseFWeel(){
        return run(()->{top.set(-0.2); bott.set(-0.2);});
    }

    public Command moveTo(double vtop, double vbott){
        double[] v  =new double[]{vtop, vbott};
        return runOnce(()->{
            SmartDashboard.putNumber("Shoulder Goal", v[0]);
            SmartDashboard.putNumber("Joint Goal", v[1]);
        }).andThen(run(
            () -> {GoTo(v[0], v[1]);
            double[] getVelocity = this.getVelocity();
            })
        );
    }
    public Command moveTill(double vtop, double vbott){
        double[] v  =new double[]{vtop, vbott};
        return runOnce(()->{
            SmartDashboard.putNumber("top Goal", v[0]);
            SmartDashboard.putNumber("bott Goal", v[1]);
        }).andThen(run(
            () -> {GoTo(v[0], v[1]);
            double[] getVelocity = this.getVelocity();
            }

        ).until(
            ()->(
                (Math.abs(getErrors(v)[0]) < Constants.Swerve.flywheel.tolerance 
                && Math.abs(getErrors(v)[1]) < Constants.Swerve.flywheel.tolerance) 
            )).withTimeout(.30)
        );
    }
    public void periodic(){
        SmartDashboard.putNumber("lywhell Vellovity", getVelocity()[0]);
    }

}
