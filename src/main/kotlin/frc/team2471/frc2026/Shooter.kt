package frc.team2471.frc2026

import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.MotorAlignmentValue
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.interpolation.InterpolatingTreeMap
import edu.wpi.first.math.interpolation.Interpolator
import edu.wpi.first.math.interpolation.InverseInterpolator
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.team2471.frc2026.AimUtils.toExitVelocity
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.team2471.frc.lib.control.Direction
import org.team2471.frc.lib.control.LoopLogger
import org.team2471.frc.lib.control.commands.finallyRun
import org.team2471.frc.lib.control.commands.onlyRunWhileFalse
import org.team2471.frc.lib.control.commands.onlyRunWhileTrue
import org.team2471.frc.lib.control.commands.parallelCommand
import org.team2471.frc.lib.control.commands.runCommand
import org.team2471.frc.lib.control.commands.runOnceCommand
import org.team2471.frc.lib.control.dPad
import org.team2471.frc.lib.ctre.addFollower
import org.team2471.frc.lib.ctre.applyConfiguration
import org.team2471.frc.lib.ctre.brakeMode
import org.team2471.frc.lib.ctre.coastMode
import org.team2471.frc.lib.ctre.currentLimits
import org.team2471.frc.lib.ctre.d
import org.team2471.frc.lib.ctre.i
import org.team2471.frc.lib.ctre.inverted
import org.team2471.frc.lib.ctre.loggedTalonFX.LoggedTalonFX
import org.team2471.frc.lib.ctre.motionMagic
import org.team2471.frc.lib.ctre.p
import org.team2471.frc.lib.ctre.remoteCANCoder
import org.team2471.frc.lib.ctre.s
import org.team2471.frc.lib.units.absoluteValue
import org.team2471.frc.lib.units.asFeet
import org.team2471.frc.lib.units.asMeters
import org.team2471.frc.lib.units.asMetersPerSecond
import org.team2471.frc.lib.units.asRadiansPerSecond
import org.team2471.frc.lib.units.asRotation2d
import org.team2471.frc.lib.units.asVolts
import org.team2471.frc.lib.units.cos
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.feet
import org.team2471.frc.lib.units.inches
import org.team2471.frc.lib.units.radians
import org.team2471.frc.lib.units.rotations
import org.team2471.frc.lib.units.rotationsPerSecond
import org.team2471.frc.lib.units.seconds
import org.team2471.frc.lib.units.sin
import org.team2471.frc.lib.units.volts
import org.team2471.frc.lib.units.voltsPerSecond
import org.team2471.frc.lib.util.angleTo
import org.team2471.frc.lib.util.isReal
import org.team2471.frc.lib.util.isSim
import kotlin.math.abs
import kotlin.math.cos

object Shooter: SubsystemBase("Shooter") {
    val table = NetworkTableInstance.getDefault().getTable("Shooter")

    // feet, rot/s (of the wheel not the motor) (in an ideal condition. need to divide by SHOOTER_EFFICIENCY)
    val hubSpeedCurve = InterpolatingTreeMap(InverseInterpolator.forDouble(), Interpolator.forDouble()).apply {
        put(3.0, 42.6)
        put(4.0, 43.31)
        put(5.0, 44.02)
        put(6.0, 45.085)
        put(7.0, 46.15)
        put(8.0, 47.606)
        put(9.0, 48.178)
        put(10.0, 48.798)
        put(11.0, 49.464)
        put(12.0, 50.176)
        put(13.0, 50.931)
        put(14.0, 53.605)
        put(15.0, 55.812)
        put(16.0, 59.0)
        put(17.0, 61.0)
        put(18.0, 62.0)
    }
    // feet, degrees
    val hubAngleCurve = InterpolatingTreeMap(InverseInterpolator.forDouble(), Interpolator.forDouble()).apply {
        put(3.0, 86.142)
        put(4.0, 84.743)
        put(5.0, 81.536)
        put(6.0, 79.592)
        put(7.0, 76.685)
        put(8.0, 73.818)
        put(9.0, 71.993)
        put(10.0, 70.213)
        put(11.0, 68.478)
        put(12.0, 66.791)
        put(13.0, 65.151)
        put(14.0, 64.56)
        put(15.0, 64.018)
        put(16.0, 63.026)
        put(17.0, 62.081)
        put(18.0, 61.621)

    }

    //feet, s
//    val hubTimeCurve = InterpolatingTreeMap(InverseInterpolator.forDouble(), Interpolator.forDouble()).apply {
//        put(3.0, 0.27)
//        put(4.0, 0.3)
//        put(5.0, 1.2)
//        put(6.0, 1.22)
//        put(7.0, 1.24)
//        put(8.0, 1.25)
//        put(9.0, 1.26)
//        put(10.0, 1.28)
//        put(11.0, 1.29)
//        put(12.0, 1.3)
//        put(13.0, 1.31)
//        put(14.0, 1.31)
//        put(15.0, 1.32)
//        put(16.0, 1.32)
//        put(17.0, 1.32)
//        put(18.0, 1.32)
//    }

    // feet, rot/s (of the wheel not the motor) (in an ideal condition. need to divide by SHOOTER_EFFICIENCY)
    val floorSpeedCurve = InterpolatingTreeMap(InverseInterpolator.forDouble(), Interpolator.forDouble()).apply {
        put(5.0, 29.79)
        put(6.0, 30.456)
        put(7.0, 31.224)
        put(8.0, 32.089)
        put(9.0, 33.635)
        put(10.0, 35.328)
        put(11.0, 36.215)
        put(12.0, 37.457)
        put(13.0, 38.811)
        put(14.0, 40.222)
        put(15.0, 41.683)
        put(16.0, 43.191)
        put(17.0, 44.741)
        put(18.0, 46.464)
        put(19.0, 48.289)
        put(20.0, 49.657)
        put(21.0, 51.403)
        put(22.0, 53.098)
        put(23.0, 54.815)
        put(24.0, 56.554)
        put(25.0, 58.311)
        put(26.0, 60.719)
        put(27.0, 62.17)
        put(28.0, 63.681)
        put(29.0, 65.499)
        put(30.0, 67.33)
        put(31.0, 69.172)
        put(32.0, 71.502)
        put(33.0, 73.465)
        put(34.0, 75.305)
        put(35.0, 77.357)
        put(36.0, 79.649)
        put(37.0, 80.843)
        put(38.0, 83.108)
        put(39.0, 84.092)
        put(40.0, 86.862)
    }
    // feet, degrees
    val floorAngleCurve = InterpolatingTreeMap(InverseInterpolator.forDouble(), Interpolator.forDouble()).apply {
        put(5.0, 71.303)
        put(6.0, 67.898)
        put(7.0, 64.649)
        put(8.0, 61.566)
        put(9.0, 58.406)
        put(10.0, 55.517)
        put(11.0, 53.092)
        put(12.0, 50.741)
        put(13.0, 48.54)
        put(14.0, 46.49)
        put(15.0, 44.582)
        put(16.0, 42.804)
        put(17.0, 41.149)
        put(18.0, 39.621)
        put(19.0, 38.214)
        put(20.0, 36.83)
        put(21.0, 35.587)
        put(22.0, 34.411)
        put(23.0, 33.308)
        put(24.0, 32.274)
        put(25.0, 31.301)
        put(26.0, 30.542)
        put(27.0, 29.6)
        put(28.0, 28.712)
        put(29.0, 27.944)
        put(30.0, 27.219)
        put(31.0, 26.532)
        put(32.0, 26.014)
        put(33.0, 25.426)
        put(34.0, 24.832)
        put(35.0, 24.324)
        put(36.0, 23.907)
        put(37.0, 23.206)
        put(38.0, 22.827)
        put(39.0, 22.105)
        put(40.0, 21.909)
    }

    val shooterShootingSpeedEntry = table.getEntry("Shooter Shooting Speed")
    val doAutoShootEntry = table.getEntry("Do Auto Shoot")

    val shooterShootingSpeed: Double get() = shooterShootingSpeedEntry.getDouble(40.0)
    val doAutoShoot: Boolean get() = doAutoShootEntry.getBoolean(true)


    val shooterMotor = LoggedTalonFX(Falcons.SHOOTER_0, CANivores.TURRET_CAN)
    val shooterMotorFollower = LoggedTalonFX(Falcons.SHOOTER_1, CANivores.TURRET_CAN)
    val hoodMotor = LoggedTalonFX(Falcons.SHOOTER_HOOD, CANivores.TURRET_CAN)
    val hoodEncoder = CANcoder(CANCoders.HOOD, CANivores.TURRET_CAN)

    val WHEEL_DIAMETER = 4.0.inches

    const val SHOOTER_GEAR_RATIO = 18.0/22.0

    // seconds
    const val HOOD_DOWN_TIME = 0.75

    @get:AutoLogOutput(key = "Shooter/Shooter Angular Velocity Setpoint")
    var shooterVelocitySetpoint: AngularVelocity = 0.0.rotationsPerSecond
        set(value) {
            field = value.coerceAtLeast(0.0.rotationsPerSecond)// / SHOOTER_GEAR_RATIO
            if (field > 0.0.rotationsPerSecond) {
                shooterMotor.setControl(VelocityVoltage(field))
            } else {
                shooterMotor.setControl(MotionMagicVoltage(0.0))
            }
        }

    @get:AutoLogOutput(key = "Shooter/Hood Feedforward")
    val hoodFeedforward: Double get() = 0.2//hoodAngle.cos() * 0.2

    // ball trajectory angle
    @get:AutoLogOutput(key = "Shooter/Hood Angle Setpoint")
    var hoodAngleSetpoint: Angle = hoodAngle
        set(value) {
            field = value.coerceIn(0.0.degrees, 44.0.degrees)
            if (field == 0.0.degrees && hoodAngle > 5.0.degrees) {
                hoodMotor.setControl(PositionVoltage(field).withFeedForward(hoodFeedforward))
            } else {
                hoodMotor.setControl(MotionMagicVoltage(field).withFeedForward(hoodFeedforward))
            }
        }

    @get:AutoLogOutput(key = "Shooter/Hood Angle")
    val hoodAngle: Angle get() = hoodMotor.position.valueAsDouble.rotations

    @get:AutoLogOutput(key = "Shooter/Hood Encoder Angle")
    val hoodEncoderAngle: Angle get() = hoodEncoder.position.value

    @get:AutoLogOutput(key = "Shooter/Shooter Angular Velocity")
    val shooterVelocity: AngularVelocity
        get() = shooterMotor.velocity.valueAsDouble.rotationsPerSecond// * SHOOTER_GEAR_RATIO

    @get:AutoLogOutput(key = "Shooter/Shooter Current")
    val shooterCurrent: Double get() = shooterMotor.supplyCurrent.valueAsDouble
    @get:AutoLogOutput(key = "Shooter/Shooter Motor Supply Voltage")
    val shooterSupplyVoltage: Double get() = shooterMotor.supplyVoltage.valueAsDouble
    @get:AutoLogOutput(key = "Shooter/Shooter Motor Voltage")
    val shooterMotorVoltage: Double get() = shooterMotor.motorVoltage.valueAsDouble

    @get:AutoLogOutput(key = "Shooter/Hood Current")
    val hoodCurrent: Double get() = hoodMotor.supplyCurrent.valueAsDouble

    // degrees
    const val HOOD_STOW_SETPOINT = 0.0

    const val BALL_ANGLE_AT_HOOD_ZERO = 90.0

    @get:AutoLogOutput(key = "Shooter/Hood error distance")
    val hoodErrorDistance get() = abs(AimUtils.distanceToTarget.asFeet * sin(hoodMotor.closedLoopError.valueAsDouble.radians))

    @get:AutoLogOutput(key = "Shooter/Velocity error distance")
    val velocityErrorDistance get() = abs((if (AimUtils.isAimingAtGoal) AimUtils.MEASURED_SHOT_AIRTIME * cos(hubAngleCurve.get(AimUtils.distanceToTarget.asFeet)) else AimUtils.PASS_AIRTIME * cos(floorAngleCurve.get(AimUtils.distanceToTarget.asFeet))) * shooterMotor.closedLoopError.valueAsDouble * WHEEL_DIAMETER.asMeters * Math.PI * 0.5)

//    @get:AutoLogOutput(key = "Shooter/Requested voltage")
//    var requestedVoltage = 0.0

//    val shooterController = PDVelocityController(0.1, 0.0, 0.1 * 6.0/7.0, true)

    var fuel: MutableList<FuelSim> = mutableListOf()
    var fuel2: MutableList<FuelSim> = mutableListOf()


    @get:AutoLogOutput(key = "Shooter/raw ramped up")
    val rawRampedUp: Boolean get() = (shooterVelocity - shooterVelocitySetpoint).absoluteValue() < 5.0.rotationsPerSecond

    var rampedUpDebouncer = Debouncer(0.1, Debouncer.DebounceType.kFalling)

    @get:AutoLogOutput(key = "Shooter/Ramped up")
    val rampedUp: Boolean get() = rampedUpDebouncer.calculate(rawRampedUp)

    @get:AutoLogOutput(key = "Shooter/Ramped up")
    val rampedUpPassing: Boolean get() = (shooterVelocity - shooterVelocitySetpoint).absoluteValue() < 15.0.rotationsPerSecond

    @get:AutoLogOutput(key = "Shooter/isShooting")
    var isShooting = false
    var i = 0

    init {
        shooterMotor.configSim(DCMotor.getKrakenX60(2), 0.1)
        hoodMotor.configSim(DCMotor.getKrakenX60(1), 0.005)

        if (!shooterShootingSpeedEntry.exists()) shooterShootingSpeedEntry.setDouble(shooterShootingSpeed)
        shooterShootingSpeedEntry.setPersistent()

        doAutoShootEntry.setBoolean(true)

        shooterMotor.applyConfiguration {
            currentLimits(39.0, 50.0, 1.0)
            coastMode()

            Feedback.withSensorToMechanismRatio(1.0/1.5)

            inverted(InvertedValue.Clockwise_Positive)

            p(if (isReal) 0.35 else 4000.0)
            i(if (isReal) 0.3 else 0.0)
//            d(0.0)
//            s(0.0, StaticFeedforwardSignValue.UseVelocitySign)

            MotionMagic.MotionMagicAcceleration = 30.0

            //Bang bang torque
//            p(99999999.9)
//            TorqueCurrent.PeakForwardTorqueCurrent = 40.0
//            TorqueCurrent.PeakReverseTorqueCurrent = 0.0
        }
        shooterMotor.addFollower(shooterMotorFollower, MotorAlignmentValue.Opposed)

        hoodMotor.applyConfiguration {
            currentLimits(25.0, 30.0, 1.0)
            inverted(true)
            brakeMode()
            s(0.05, StaticFeedforwardSignValue.UseClosedLoopSign)
            p(if (isReal) 60.0 else 60.0)
            d(if (isReal) 0.0 else 4.0)

            motionMagic(0.75, 5.0)

            remoteCANCoder(hoodEncoder.deviceID, 9.64285714285714)
        }

//        GlobalScope.launch {
//            periodic(0.01) {
//                requestedVoltage = shooterController.updateVoltage(shooterVelocitySetpoint.asRotationsPerSecond, shooterVelocity.asRotationsPerSecond).coerceIn(0.0, 13.0)
////                shooterMotor.setControl(VoltageOut(requestedVoltage))
//            }
//        }
    }



    override fun periodic() {
        LoopLogger.record("b4 Shooter periodic")
        if (isSim) {
            if (isShooting) {
                if (i > 1) {
                    i = 0
                    shootSimulatedFuelWithMotors()
                } else {
                    i++
                }
            }
            fuel.forEach { it.update() }
            logFuel("fuel", *fuel.toTypedArray())
            fuel.removeFuel()

            fuel2.forEach { it.update() }
            logFuel("fuel2", *fuel2.toTypedArray())
            fuel2.removeFuel()
        }

//        shooterMotor.setControl(VoltageOut(shooterController.updateVoltage(shooterAngularVelocitySetpoint.asRotationsPerSecond, shooterAngularVelocity.asRotationsPerSecond)))
        LoopLogger.record("Shooter periodic")
    }

    fun default(): Command = runCommand(this) {
        if ((doAutoShoot && !Drive.cameraDisconnected) && Drive.useAprilTags && AimUtils.isAimingAtGoal) {
            if (FieldManager.inScoringZone && !FieldManager.inTrenchArea && AimUtils.distanceToTarget < 13.0.feet && FieldManager.shouldShoot) {
                shootLoop()
            } else {
                isShooting = false
                if (Intake.intakeState != Intake.IntakeState.INTAKING) {
                    Spindexer.currentState = Spindexer.State.OFF
                }
                hoodAngleSetpoint = HOOD_STOW_SETPOINT.degrees
            }
            if (FieldManager.shouldShoot && FieldManager.inScoringZone) {
                rampUpLoop()
            }
        } else {
            hoodAngleSetpoint = HOOD_STOW_SETPOINT.degrees
            shooterVelocitySetpoint = 80.0.rotationsPerSecond
            isShooting = false
            if (Intake.intakeState != Intake.IntakeState.INTAKING) {
                Spindexer.currentState = Spindexer.State.OFF
            }
        }
    }


    fun shootOrRamp(): Command {
        return parallelCommand(
            runCommand(Shooter) {
                rampUpLoop()
            },
            runCommand {
                shootLoop()
            }.onlyRunWhileTrue { OI.driverController.rightTriggerAxis >= 0.1 || OI.driverController.dPad == Direction.RIGHT }.repeatedly(),
            runCommand {
                isShooting = false
                Spindexer.currentState = Spindexer.State.OFF
                hoodAngleSetpoint = HOOD_STOW_SETPOINT.degrees
            }.onlyRunWhileFalse { OI.driverController.rightTriggerAxis >= 0.1 || OI.driverController.dPad == Direction.RIGHT }.repeatedly()
        ).finallyRun {
            isShooting = false
            Spindexer.currentState = Spindexer.State.OFF
            hoodAngleSetpoint = HOOD_STOW_SETPOINT.degrees
        }
    }


    fun shoot(ignoreRampUp: Boolean = false): Command = runCommand(Shooter) {
        shootLoop(ignoreRampUp)
        rampUpLoop()
    }.finallyRun {
        isShooting = false
        Spindexer.currentState = Spindexer.State.OFF
        hoodAngleSetpoint = HOOD_STOW_SETPOINT.degrees
    }

    fun rampUpLoop() {
        shooterVelocitySetpoint = AimUtils.getShooterRPS()
    }

    fun shootLoop(ignoreRampUp: Boolean = false) {
//        println("Shoot Loop!!!")
        if (!FieldManager.inTrenchArea && (!Turret.isTurretWrapping || Turret.disableTurret) && (((rampedUp || ignoreRampUp) && AimUtils.isAimingAtGoal) || (rampedUpPassing && !AimUtils.isAimingAtGoal)) && (FieldManager.shouldShoot || !AimUtils.isAimingAtGoal)) {
            isShooting = true
            Spindexer.currentState = Spindexer.State.ON
        } else {
            isShooting = false
            Spindexer.currentState = Spindexer.State.OFF
        }

        if (!FieldManager.inTrenchArea) {
            hoodAngleSetpoint = (
                if (AimUtils.isAimingAtGoal)
                    BALL_ANGLE_AT_HOOD_ZERO - hubAngleCurve.get(AimUtils.distanceToTarget.asFeet)
                else
                    BALL_ANGLE_AT_HOOD_ZERO - floorAngleCurve.get(AimUtils.distanceToTarget.asFeet)
            ).degrees
        } else {
            hoodAngleSetpoint = HOOD_STOW_SETPOINT.degrees
        }

    }


    fun rampUp(): Command = runCommand(Shooter) {
        rampUpLoop()
    }

    fun rampDown(): Command = runOnceCommand(Shooter) {
        shooterVelocitySetpoint = 0.0.rotationsPerSecond
    }

    fun shootSimulatedFuel() {
        val exitVelocity = hubSpeedCurve.get(AimUtils.distanceToTarget.asFeet) / SHOOTER_GEAR_RATIO
        val exitAngle = hubAngleCurve.get(AimUtils.distanceToTarget.asFeet).degrees
        val angleToTarget = Turret.turretTranslation.angleTo(AimUtils.aimTarget)
        val velocity2d = Translation2d(exitVelocity * exitAngle.cos(), 0.0).rotateBy(angleToTarget.asRotation2d)
        val turretVelocity = Translation2d(Turret.turretOffsetFromCenter.x * Drive.gyroYawRate.asRadiansPerSecond, Turret.turretOffsetFromCenter.y * Drive.gyroYawRate.asRadiansPerSecond).rotateBy(Drive.heading) + Drive.velocity
        fuel.add(FuelSim(
            Translation3d(Turret.turretTranslation.x, Turret.turretTranslation.y, 0.4),
            Translation3d(velocity2d.x + turretVelocity.x, velocity2d.y + turretVelocity.y, exitVelocity * exitAngle.sin())
        ))
    }

    fun shootSimulatedFuelWithMotors() {
        val exitVelocity = shooterVelocity.toExitVelocity().asMetersPerSecond
        val exitAngle = hoodAngle
//        val angleToTarget = AimUtils.aimTarget.angleTo(Turret.turretPose)
        val velocity2d = Translation2d(exitVelocity * exitAngle.cos(), 0.0).rotateBy(Turret.fieldCentricAngle.asRotation2d)
        val turretVelocity = Translation2d(Turret.turretOffsetFromCenter.x * Drive.gyroYawRate.asRadiansPerSecond, Turret.turretOffsetFromCenter.y * Drive.gyroYawRate.asRadiansPerSecond).rotateBy(Drive.heading) + Drive.velocity
        fuel.add(FuelSim(
            Translation3d(Turret.turretTranslation.x, Turret.turretTranslation.y, 0.4),
            Translation3d(velocity2d.x + turretVelocity.x, velocity2d.y + turretVelocity.y, exitVelocity * exitAngle.sin())
        ))
    }

    val sysIDShooterRoutine = SysIdRoutine(
        SysIdRoutine.Config(
            1.0.voltsPerSecond,
            7.0.volts,
            5.0.seconds
        ) { state: SysIdRoutineLog.State ->
            SignalLogger.writeString("SysIdShooterLeft_State", state.toString())
            Logger.recordOutput("SysIdShooterLeft_State", state.toString())
            Logger.recordOutput("Shooter_Left_Position", shooterMotor.position.valueAsDouble)
            Logger.recordOutput("Shooter_Left_Velocity", shooterMotor.velocity.valueAsDouble)
        },
        SysIdRoutine.Mechanism({ output: Voltage ->
            shooterMotor.setControl(VoltageOut(output.asVolts))
            /* also log the requested output for SysId */
            Logger.recordOutput("Shooter_Left_Voltage", output.asVolts + 0.0001 * Math.random())
        }, null, this)
    )
}