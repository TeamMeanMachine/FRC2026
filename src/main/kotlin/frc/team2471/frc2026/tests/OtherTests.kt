package frc.team2471.frc2026.tests

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
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
import org.team2471.frc.lib.units.asDegrees
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.unWrap

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

fun intakeDeployTest() = sequenceCommand(
    runOnce{ Intake.stow() },
    waitCommand(2.0),
    runOnce{ Intake.deploy() },
    waitCommand(2.0),
    runOnce{ Intake.stow() },
    waitCommand(2.0),
    runOnce{ println("Intake deploy tested") },
)

fun intakeRollerTest() = sequenceCommand(
    runOnce{ Intake.deploy() },
    waitCommand(2.0),
    runOnce{ Intake.velocitySetpoint = 10.0 },
    waitCommand(2.0),
    runOnce{ Intake.velocitySetpoint = 100.0 },
    waitCommand(2.0),
    runOnce{  Intake.velocitySetpoint = 0.0 },
    waitCommand(2.0),
    runOnce{ println("Intake roller tested") },
)

fun turretTest(): Command {
    fun localToFieldCentric(local: Angle) = ((local) + Drive.heading.measure).unWrap(Turret.fieldCentricAngle)

    return sequenceCommand(
        runOnce{ Turret.fieldCentricSetpoint = localToFieldCentric(0.0.degrees) },
        waitCommand(2.0),
        runOnce{ Turret.fieldCentricSetpoint = localToFieldCentric(90.0.degrees) },
        waitCommand(2.0),
        runOnce{ Turret.fieldCentricSetpoint = localToFieldCentric(180.0.degrees) },
        waitCommand(2.0),
        runOnce{ Turret.fieldCentricSetpoint = localToFieldCentric(Turret.TURRET_TOP_LIMIT) },
        waitCommand(2.0),
        runOnce{ Turret.fieldCentricSetpoint = localToFieldCentric(-90.0.degrees) },
        waitCommand(2.0),
        runOnce{ Turret.fieldCentricSetpoint = localToFieldCentric(-180.0.degrees) },
        waitCommand(2.0),
        runOnce{ Turret.fieldCentricSetpoint = localToFieldCentric(Turret.TURRET_BOTTOM_LIMIT) },
        waitCommand(2.0),
        runOnce{ Turret.fieldCentricSetpoint = localToFieldCentric(0.0.degrees) },
        waitCommand(2.0),
        runOnce{ println("Turret tested") }
    )
}

fun spindexerAndShooterTest() = sequenceCommand(
    runOnce{ Spindexer.currentState = Spindexer.State.ON },
    waitCommand(2.0),
    runOnce{ Spindexer.currentState = Spindexer.State.AGITATING },
    waitCommand(2.0),
    runOnce{ Spindexer.currentState = Spindexer.State.OFF },
    waitCommand(1.0),
    Shooter.shoot().withTimeout(4.0)
)

fun fullSystemTest() = sequenceCommand(
    turretTest(),
    intakeDeployTest(),
    intakeRollerTest(),
    spindexerAndShooterTest()
)