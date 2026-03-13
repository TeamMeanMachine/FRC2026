package frc.team2471.frc2026.tests

import frc.team2471.frc2026.AimUtils
import frc.team2471.frc2026.AimUtils.printShooterCurves
import frc.team2471.frc2026.Drive
import frc.team2471.frc2026.Turret
import frc.team2471.frc2026.Intake
import frc.team2471.frc2026.Shooter
import frc.team2471.frc2026.Spindexer
import org.team2471.frc.lib.control.commands.runOnce
import org.team2471.frc.lib.control.commands.runOnceCommand
import org.team2471.frc.lib.control.commands.sequenceCommand
import org.team2471.frc.lib.control.commands.waitCommand
import org.team2471.frc.lib.coroutines.delay
import org.team2471.frc.lib.units.asDegrees
import org.team2471.frc.lib.units.degrees

// Prints the hub curves using a gradle task. Needs a main function in a class so I put it here.
object PrintHubCurves {
    @JvmStatic
    fun main(args: Array<String>) {
        // constant time
        printShooterCurves(AimUtils.HUB_HEIGHT - Turret.turretHeight, 3..18, AimUtils.MEASURED_SHOT_AIRTIME)

        // constant(ish) exit velocity
//        printShooterCurves(AimUtils.HUB_HEIGHT - Turret.turretHeight, 3..18, Pair(70.0, 78.0))
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

fun intakeTest() = sequenceCommand(
    runOnce{Intake.stow()},
    waitCommand(2.0),
    runOnce{Intake.deploy()},
    waitCommand(2.0),
    runOnce{Intake.intakeState = Intake.IntakeState.INTAKING},
    waitCommand(2.0),
    runOnce{Intake.intakeState = Intake.IntakeState.SPITTING},
    waitCommand(2.0),
    runOnce{Intake.stow()},
    waitCommand(2.0),
    runOnce{println("Intake tested")},
    )

// TODO: NOT USE FIELD CENTRIC SETPOINT
fun turretTest() = sequenceCommand(
    runOnce {Turret.setTurretOffset(270.0.degrees)},
    waitCommand(2.0),
    runOnce{Turret.setTurretOffset(-540.0.degrees)},
    waitCommand(2.0),
    runOnce{Turret.setTurretOffset(270.0.degrees)},
    waitCommand(2.0),
    runOnce{Turret.setTurretOffset(-90.0.degrees)},
    waitCommand(2.0),
    runOnce{Turret.setTurretOffset(180.0.degrees)},
    waitCommand(2.0),
    runOnce{Turret.setTurretOffset(-90.0.degrees) },
    runOnce{println("Turret tested")}
)
fun spindexerAndShooterTest() = sequenceCommand(
    runOnce{Spindexer.State.ON},
    waitCommand(2.0),
    runOnce{ Shooter.shoot()}
)