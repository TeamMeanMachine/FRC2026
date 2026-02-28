package frc.team2471.frc2026.tests

import edu.wpi.first.wpilibj2.command.Command
import frc.team2471.frc2026.AimUtils
import frc.team2471.frc2026.AimUtils.printShooterCurves
import frc.team2471.frc2026.Turret
import frc.team2471.frc2026.Intake
import org.team2471.frc.lib.control.commands.runCommand
import org.team2471.frc.lib.control.commands.runOnce
import org.team2471.frc.lib.units.asDegrees

// Prints the hub curves using a gradle task. Needs a main function in a class so I put it here.
object PrintHubCurves {
    @JvmStatic
    fun main(args: Array<String>) {
        // constant time
        printShooterCurves(AimUtils.HUB_HEIGHT - Turret.turretHeight, 3..18, AimUtils.SHOT_AIRTIME)

        // constant(ish) exit velocity
//        printShooterCurves(AimUtils.HUB_HEIGHT - Turret.turretHeight, 3..18, Pair(7.0, 9.0))
    }
}

// Prints the pass curves using a gradle task. Needs a main function in a class so I put it here.
object PrintPassCurves {
    @JvmStatic
    fun main(args: Array<String>) {
        printShooterCurves( -Turret.turretHeight, 5..40, AimUtils.PASS_AIRTIME)
    }
}

fun zeroTurretEncoders() = runCommand() {
    Turret.encoder1Offset.setDouble(Turret.rawEncoder1AbsolutePosition.asDegrees)
    Turret.encoder2Offset.setDouble(Turret.rawEncoder2AbsolutePosition.asDegrees)
    Turret.turretPigeon.setYaw(0.0)
    Turret.turretMotor.setPosition(0.0)
}
fun intakeTest() = runCommand() {
    Intake.stow()
    Intake.deploy()
    Intake.IntakeState = Intake.IntakeState.INTAKING
}