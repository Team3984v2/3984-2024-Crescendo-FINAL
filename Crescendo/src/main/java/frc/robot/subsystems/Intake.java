package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.intake.intakeArm;
import frc.robot.Constants.Swerve.intake.intakeMotor;
import frc.robot.subsystems.LED;;

public class Intake extends SubsystemBase{
    
    private CANSparkMax lolIntakeArm;
    private RelativeEncoder intakeArmEncoder;
    private SparkPIDController intakeArmPID;
    private Spark intakeMotorL;
    private Spark intakeMotorR;
    //private DigitalInput funnyButton;
    //private LED funnyLight;

    public Intake(){

        lolIntakeArm = new CANSparkMax(
            intakeArm.rotMotorID,
            MotorType.kBrushless
        );

        intakeMotorL = new Spark(
            intakeMotor.IDL
        );
        intakeMotorR = new Spark(
            intakeMotor.IDR
        );

        //funnyButton = new DigitalInput(0);

        //funnyLight = new LED(1);


        lolIntakeArm.restoreFactoryDefaults();
        lolIntakeArm.setIdleMode(IdleMode.kBrake);

        //Initializes encoder for intake's arm
        intakeArmEncoder = lolIntakeArm.getEncoder(
            SparkRelativeEncoder.Type.kHallSensor, 
            42
        );


        // Set Conversion factor for Encoder -> degrees
        intakeArmEncoder.setPositionConversionFactor(
            360.0/intakeArm.gearRatio 
        ); 
        // Set Velocity Conversion factor -> degrees/second
        intakeArmEncoder.setVelocityConversionFactor(
            (360 / intakeArm.gearRatio) / 60.0  
        );
        // Set the position to zero
        intakeArmEncoder.setPosition(0);

        //Initialize arm PID
        intakeArmPID = lolIntakeArm.getPIDController();
        intakeArmPID.setP(Constants.Swerve.intake.intakeArm.kP);
        intakeArmPID.setI(Constants.Swerve.intake.intakeArm.kI);
        intakeArmPID.setD(Constants.Swerve.intake.intakeArm.kD);

        lolIntakeArm.burnFlash();
    }
    
    public Command Out(){
         return run(()->{
            intakeMotorL.set(-1); 
            intakeMotorR.set(1);
         });
    }
    public Command In(){
        return run(()->{
            intakeMotorL.set(1);
            intakeMotorR.set(-1);
        });
    }
    public Command Stop(){
        return run(()->{
            intakeMotorL.stopMotor();
            intakeMotorR.stopMotor();
        });
    }

    public Rotation2d getPos(){
        Rotation2d posIntakeArm = Rotation2d.fromDegrees(intakeArmEncoder.getPosition()); 
        return posIntakeArm;
    }
    public Rotation2d getErrors(Rotation2d goal){
        Rotation2d currPos =getPos();
        double error = currPos.getDegrees() - goal.getDegrees();
        return Rotation2d.fromDegrees(error);
    }
    // Run PID
    public void GoTo(Rotation2d goal){
        intakeArmPID.setReference(
            goal.getDegrees(), 
            ControlType.kPosition, 0
        );
        
    }
    // Convert PID method into a runable command
    // if button is pressed, go down to goal, until within setpoint or when button stopped pressing, when button at setpoint, start intake and go until 
    public Command moveTo(Rotation2d goal, boolean intake){
        return runOnce(()->{
            SmartDashboard.putNumber("Goal", goal.getDegrees());
        }).andThen(run(
            () -> GoTo(goal) 
        ).withTimeout(1)/*until(
            ()->(
                Math.abs(getErrors(goal).getDegrees()) < Constants.Swerve.intake.tolerance 
                && Math.abs(getErrors(goal).getDegrees()) < Constants.Swerve.intake.tolerance
            )
        ).andThen((intake)? In(): Stop())*/
        );
    }
    public void periodic(){
        SmartDashboard.putNumber("IntakePos", getPos().getDegrees());
        /*if(funnyButton.get()){
            funnyLight.setState(true);
        }else{
            funnyLight.setState(false);
        }*/
    }
}
