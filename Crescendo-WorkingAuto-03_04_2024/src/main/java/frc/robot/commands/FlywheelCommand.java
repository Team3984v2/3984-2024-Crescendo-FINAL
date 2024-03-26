package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;

public class FlywheelCommand extends Command{
    private Flywheel flywheel;
    private DoubleSupplier trigger;
    private BooleanSupplier amp;
    private BooleanSupplier speaker;

    public FlywheelCommand(
        Flywheel flywheel,
        DoubleSupplier trigger,
        BooleanSupplier amp,
        BooleanSupplier speaker){
            this.flywheel = flywheel;
            addRequirements(flywheel);
            this.trigger = trigger;
            this.amp = amp;
            this.speaker = speaker;
    }

    @Override
    public void execute(){
        double triggerValue = MathUtil.applyDeadband(trigger.getAsDouble(), .3);
        boolean toggleShooter = false;
        //Idle mode
        if (triggerValue < 0.1){
        }
        if (toggleShooter) {

        }
    }

    
}
