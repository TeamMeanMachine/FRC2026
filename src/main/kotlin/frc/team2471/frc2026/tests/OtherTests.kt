package frc.team2471.frc2026.tests

import edu.wpi.first.wpilibj2.command.Command
import frc.team2471.frc2026.AimUtils
import frc.team2471.frc2026.AimUtils.printShooterCurves
import frc.team2471.frc2026.Drive
import frc.team2471.frc2026.Turret
import frc.team2471.frc2026.Intake
import org.team2471.frc.lib.control.commands.runCommand
import org.team2471.frc.lib.control.commands.runOnce
import org.team2471.frc.lib.control.commands.runOnceCommand
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

fun zeroTurretEncoders() = runOnceCommand(Turret) {
    Turret.encoder1Offset.setDouble(Turret.rawEncoder1AbsolutePosition.asDegrees)
    Turret.encoder2Offset.setDouble(Turret.rawEncoder2AbsolutePosition.asDegrees)
    Turret.setTurretOffset(Drive.heading.measure)
    Turret.zeroTurretMotor()

    println("Zeroed turret encoders")
}
fun intakeTest() = runCommand() {
    Intake.stow()
    Intake.deploy()
}