package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private RollerIO rollerIO;
    private RollerIO rollerIO2;
    private PneumaticsIO pneumaticsIO;
    
    private int shooterSpeed; // radians per second

    public ShooterSubsystem(RollerIO rollerIO, RollerIO rollerIO2, PneumaticsIO pneumaticsIO) {

        this.rollerIO = rollerIO;
        this.rollerIO2 = rollerIO2;
        this.pneumaticsIO = pneumaticsIO;

    }

}
