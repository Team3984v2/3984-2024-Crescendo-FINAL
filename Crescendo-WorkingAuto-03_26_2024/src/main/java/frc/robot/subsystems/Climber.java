package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.climber;


public class Climber extends SubsystemBase{


    private CANSparkMax climbMotor; 
    private RelativeEncoder EncoderClimb;
    private SparkPIDController ClimbPID;
    private CANSparkMax climbMotor2;
    private RelativeEncoder EncoderClimb2;
    private SparkPIDController ClimbPID2;


    public Climber() {
        // Initialize the Motors
        climbMotor = new CANSparkMax(
            climber.rotMotorID, 
            MotorType.kBrushless
        
        );
        climbMotor2 = new CANSparkMax(
            climber.rotMotorID2, 
            MotorType.kBrushless
        );
        climbMotor.setInverted(true);
        
        climbMotor2.restoreFactoryDefaults();
        climbMotor.restoreFactoryDefaults();
        climbMotor2.setIdleMode(IdleMode.kBrake);
        climbMotor.setIdleMode(IdleMode.kBrake);
        
        // Initialize the built in shoulder encoder
        EncoderClimb = climbMotor.getEncoder(
            SparkRelativeEncoder.Type.kHallSensor, 
            42
        );
        EncoderClimb2 = climbMotor2.getEncoder(
            SparkRelativeEncoder.Type.kHallSensor,
            42
        );
        
        // Set Conversion factor for Encoder -> degrees
        EncoderClimb.setPositionConversionFactor(
            360.0/climber.gearRatio
        ); 
        // Set Velocity Conversion factor -> degrees/second
        EncoderClimb.setVelocityConversionFactor(
            (360 / climber.gearRatio) / 60.0  
        );
        // Set the position to zero
        EncoderClimb.setPosition(0);
        EncoderClimb2.setPositionConversionFactor(
            360.0/climber.gearRatio 
        ); 
        // Set Velocity Conversion factor -> degrees/second
        EncoderClimb2.setVelocityConversionFactor(
            (360 / climber.gearRatio) / 60.0  //TODO
        );
        // Set the position to zero
        EncoderClimb2.setPosition(0);
        // Initialiation of PIDs
        ClimbPID = climbMotor.getPIDController();
        ClimbPID.setP(Constants.Swerve.climber.kP);
        ClimbPID.setI(Constants.Swerve.climber.kI);
        ClimbPID.setD(Constants.Swerve.climber.kD);

        ClimbPID2 = climbMotor2.getPIDController();
        ClimbPID2.setP(Constants.Swerve.climber.kP);
        ClimbPID2.setI(Constants.Swerve.climber.kI);
        ClimbPID2.setD(Constants.Swerve.climber.kD);

        // burn flash
        climbMotor.burnFlash();
        climbMotor2.burnFlash();
    }
    // Manual Controls:
    public void Up(){
        climbMotor.set(-.5);
        climbMotor2.set(.5);
    }
    public void Down(){
        climbMotor.set(.5);
        climbMotor2.set(-.5);
    }
    public void Stop(){
        climbMotor.stopMotor();
        climbMotor2.stopMotor();
    }
    public Command manualControl(BooleanSupplier up, BooleanSupplier down){
        return run(()->{
            if (up.getAsBoolean()){
               Up(); 
               System.out.print("going up");
            }
            else if (down.getAsBoolean()){
                Down();
            }
            else{
                Stop();
            }
        });

    }
    public Rotation2d[] getPos(){
        Rotation2d posClimb = Rotation2d.fromDegrees(-EncoderClimb.getPosition()); 
        Rotation2d posClimb2 = Rotation2d.fromDegrees(EncoderClimb2.getPosition()); 
        Rotation2d[] angles = new Rotation2d[]{posClimb, posClimb2};
        return angles;
    }
    public Rotation2d[] getErrors(Rotation2d[] goal){
        Rotation2d[] currPos = new Rotation2d[] {getPos()[0], getPos()[1]};
        Rotation2d[] error = new Rotation2d[] {
            Rotation2d.fromDegrees(currPos[0].getDegrees() - goal[0].getDegrees()), 
            Rotation2d.fromDegrees(currPos[1].getDegrees() - goal[1].getDegrees())
        };
        return error;
    }
    // Run PID
    public void GoTo(Rotation2d[] goal){
        ClimbPID.setReference(
            goal[0].getDegrees(), 
            ControlType.kPosition, 0
        );
        ClimbPID2.setReference(
            goal[1].getDegrees(), 
            ControlType.kPosition, 0
        );
    }
    // Convert PID method into a runable command
    public Command moveTo(Rotation2d[] goal){
        return runOnce(()->{
            SmartDashboard.putNumber("Right Goal", goal[0].getDegrees());
            SmartDashboard.putNumber("Left Goal", goal[1].getDegrees());
        }).andThen(run(
            () -> GoTo(goal)
        ).until(
            ()->(
                Math.abs(getErrors(goal)[0].getDegrees()) < Constants.Swerve.climber.tolerance &&
                Math.abs(getErrors(goal)[1].getDegrees()) < Constants.Swerve.climber.tolerance
            ))
        );
    }
   
    public void periodic(){
        SmartDashboard.putNumber("ClimbPos", getPos()[0].getDegrees());
        SmartDashboard.putNumber("ClimbPos1", getPos()[1].getDegrees());

    }

}
