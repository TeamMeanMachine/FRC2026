package frc.team2471.frc2026.tests

import edu.wpi.first.wpilibj2.command.Command
import frc.team2471.frc2026.AimUtils
import frc.team2471.frc2026.AimUtils.printShooterCurves
import frc.team2471.frc2026.Turret
import org.team2471.frc.lib.control.commands.runOnce

// Prints the hub curves using a gradle task. Needs a main function in a class so I put it here.
object PrintHubCurves {
    @JvmStatic
    fun main(args: Array<String>) {
        printShooterCurves(AimUtils.HUB_HEIGHT - Turret.turretHeight, AimUtils.SHOT_AIRTIME, 3..20)
    }
}

// Prints the pass curves using a gradle task. Needs a main function in a class so I put it here.
object PrintPassCurves {
    @JvmStatic
    fun main(args: Array<String>) {
        printShooterCurves( -Turret.turretHeight, AimUtils.PASS_AIRTIME, 5..40)
    }
}