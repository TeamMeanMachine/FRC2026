package frc.team2471.frc2026.tests

import edu.wpi.first.wpilibj2.command.Command
import frc.team2471.frc2026.AimUtils
import frc.team2471.frc2026.AimUtils.printShooterCurves
import frc.team2471.frc2026.Turret
import org.team2471.frc.lib.control.commands.runOnce

fun printHubCurves(): Command = runOnce{
    printShooterCurves(AimUtils.HUB_HEIGHT - Turret.turretHeight, AimUtils.SHOT_AIRTIME)

}
fun printPassCurves(): Command = runOnce{
    printShooterCurves( -Turret.turretHeight, AimUtils.PASS_AIRTIME)
}