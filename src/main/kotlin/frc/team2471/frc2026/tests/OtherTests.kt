package frc.team2471.frc2026.tests

import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.NeutralOut
import com.ctre.phoenix6.signals.MotorAlignmentValue
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import frc.team2471.frc2026.AimUtils
import frc.team2471.frc2026.AimUtils.printPassTimes
import frc.team2471.frc2026.AimUtils.printShooterCurves
import frc.team2471.frc2026.Drive
import frc.team2471.frc2026.Turret
import frc.team2471.frc2026.Intake
import frc.team2471.frc2026.Shooter
import frc.team2471.frc2026.Spindexer
import org.team2471.frc.lib.control.commands.*
import org.team2471.frc.lib.ctre.addFollower
import org.team2471.frc.lib.units.asDegrees
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.rotationsPerSecond
import org.team2471.frc.lib.units.unWrap

// Prints the hub curves using a gradle task. Needs a main function in a class so I put it here.
object PrintHubCurves {
    @JvmStatic
    fun main(args: Array<String>) {
        // constant time
//        printShooterCurves(AimUtils.HUB_HEIGHT - Turret.turretHeight, 3..18 step 3, AimUtils.MEASURED_SHOT_AIRTIME)

//         constant(ish) exit velocity
        printShooterCurves(AimUtils.HUB_HEIGHT - Turret.turretHeight, 3..18 step 3, Pair(40.0, 55.0))
    }
}

// Prints the pass curves using a gradle task. Needs a main function in a class so I put it here.
object PrintPassCurves {
    @JvmStatic
    fun main(args: Array<String>) {
//        printShooterCurves( -Turret.turretHeight, 5..40 step 5, 45.0.degrees)
//        printShooterCurves(-Turret.turretHeight, 5..20 step 5, Pair(50.0, 50.0))
        printPassTimes(5..20 step 5, 55.0.rotationsPerSecond)
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
    runOnce { Intake.stow() },
    waitCommand(2.0),
    runOnce { Intake.deploy() },
    waitCommand(2.0),
    runOnce { Intake.stow() },
    waitCommand(2.0),
    printlnCommand("Intake deploy tested"),
).apply {
    addRequirements(Intake, Shooter)
}

fun intakeRollerTest() = sequenceCommand(
    runOnce { Intake.deploy() },
    waitCommand(2.0),
    runOnce { Intake.velocitySetpoint = 10.0 },
    waitCommand(2.0),
    runOnce { Intake.velocitySetpoint = 100.0 },
    waitCommand(2.0),
    runOnce { Intake.velocitySetpoint = 0.0 },
    waitCommand(2.0),

    runOnce { Intake.rollerMotorFollower.setControl(DutyCycleOut(0.8)) },
    waitCommand(2.0),
    runOnce { Intake.rollerMotorFollower.setControl(NeutralOut()) },
    waitCommand(2.0),

    runOnce { Intake.rollerMotor.setControl(DutyCycleOut(0.8)) },
    waitCommand(2.0),
    runOnce { Intake.rollerMotorFollower.setControl(NeutralOut()) },
    waitCommand(2.0),

    runOnce { Intake.rollerMotorFollower.setControl(Follower(Intake.rollerMotor.deviceID, MotorAlignmentValue.Aligned)) },

    runOnce { Intake.stow() },

    printlnCommand("Intake roller tested")
).apply {
    addRequirements(Intake, Shooter)
}

fun turretTest(): Command {
    fun localToFieldCentric(local: Angle) = ((local) + Drive.heading.measure).unWrap(Turret.fieldCentricAngle)

    return sequenceCommand(
        runOnce { Turret.fieldCentricSetpoint = localToFieldCentric(0.0.degrees) },
        waitCommand(2.0),
        runOnce { Turret.fieldCentricSetpoint = localToFieldCentric(90.0.degrees) },
        waitCommand(2.0),
        runOnce { Turret.fieldCentricSetpoint = localToFieldCentric(180.0.degrees) },
        waitCommand(2.0),
        runOnce { Turret.fieldCentricSetpoint = localToFieldCentric(Turret.TURRET_TOP_LIMIT) },
        waitCommand(2.0),
        runOnce { Turret.fieldCentricSetpoint = localToFieldCentric(-90.0.degrees) },
        waitCommand(2.0),
        runOnce { Turret.fieldCentricSetpoint = localToFieldCentric(-180.0.degrees) },
        waitCommand(2.0),
        runOnce { Turret.fieldCentricSetpoint = localToFieldCentric(Turret.TURRET_BOTTOM_LIMIT) },
        waitCommand(2.0),
        runOnce { Turret.fieldCentricSetpoint = localToFieldCentric(0.0.degrees) },
        waitCommand(2.0),
        printlnCommand("Turret tested")
    ).apply {
        addRequirements(Turret, Shooter)
    }
}

fun spindexerTest() = sequenceCommand(
    runOnce { Spindexer.spinMotorVelocitySetpoint = 80.0 },
    waitCommand(2.0),
    runOnce { Spindexer.spinMotorVelocitySetpoint = 0.0 },
    waitCommand(2.0),

    runOnce { Spindexer.sidetakeMotorVelocitySetpoint = 115.0 },
    waitCommand(2.0),
    runOnce { Spindexer.sidetakeMotorVelocitySetpoint = 0.0 },
    waitCommand(2.0),

    runOnce { Spindexer.uptakeMotorVelocitySetpoint = 115.0 },
    waitCommand(2.0),
    runOnce { Spindexer.uptakeMotorVelocitySetpoint = 0.0 },
    waitCommand(2.0),


    runOnce { Spindexer.spinMotorFollower.setControl(DutyCycleOut(0.5)) },
    waitCommand(2.0),
    runOnce { Spindexer.spinMotorFollower.setControl(NeutralOut()) },
    waitCommand(2.0),

    runOnce { Spindexer.spinMotor.setControl(DutyCycleOut(0.5)) },
    waitCommand(2.0),
    runOnce { Spindexer.spinMotor.setControl(NeutralOut()) },
    waitCommand(2.0),

    runOnce { Spindexer.spinMotorFollower.setControl(Follower(Intake.rollerMotor.deviceID, MotorAlignmentValue.Aligned)) },

    printlnCommand("Spindexer tested")
).apply {
    addRequirements(Spindexer, Shooter)
}

fun shooterTest() = sequenceCommand(
    runOnceCommand(Shooter) { Shooter.shooterVelocitySetpoint = 50.0.rotationsPerSecond },
    runOnceCommand(Shooter) { Shooter.hoodAngleSetpoint = 40.0.degrees },
    waitCommand(2.0),
    runOnceCommand(Shooter) { Shooter.hoodAngleSetpoint = 0.0.degrees },
    runOnceCommand(Shooter) { Shooter.shooterVelocitySetpoint = 0.0.rotationsPerSecond },
    waitCommand(2.0),


    runOnce { Shooter.shooterMotorFollower.setControl(DutyCycleOut(-0.5)) },
    waitCommand(2.0),
    runOnce { Shooter.shooterMotorFollower.setControl(NeutralOut()) },
    waitCommand(2.0),

    runOnce { Shooter.shooterMotor.setControl(DutyCycleOut(0.5)) },
    waitCommand(2.0),
    runOnce { Shooter.shooterMotor.setControl(NeutralOut()) },
    waitCommand(2.0),

    runOnce { Shooter.shooterMotorFollower.setControl(Follower(Intake.rollerMotor.deviceID, MotorAlignmentValue.Opposed)) },

    printlnCommand("Shooter tested")
).apply {
    addRequirements(Shooter)
}

fun fullSystemTest() = sequenceCommand(
    turretTest(),
    intakeDeployTest(),
    intakeRollerTest(),
    spindexerTest(),
    shooterTest(),
    printlnCommand("Tests complete")
).apply {
    addRequirements(Intake, Shooter, Spindexer, Turret)
}