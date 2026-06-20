package frc.team2471.frc2026

import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.NeutralOut
import com.ctre.phoenix6.controls.PositionVoltage
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
import frc.team2471.frc2026.AimUtils.shooterEfficiency
import frc.team2471.frc2026.AimUtils.toExitVelocity
import frc.team2471.frc2026.Robot.isCompBot
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.team2471.frc.lib.control.LoopLogger
import org.team2471.frc.lib.control.b
import org.team2471.frc.lib.control.commands.finallyRun
import org.team2471.frc.lib.control.commands.onlyRunWhileFalse
import org.team2471.frc.lib.control.commands.onlyRunWhileTrue
import org.team2471.frc.lib.control.commands.parallelCommand
import org.team2471.frc.lib.control.commands.runCommand
import org.team2471.frc.lib.control.rightStickButton
import org.team2471.frc.lib.ctre.addFollower
import org.team2471.frc.lib.ctre.applyConfiguration
import org.team2471.frc.lib.ctre.brakeMode
import org.team2471.frc.lib.ctre.coastMode
import org.team2471.frc.lib.ctre.currentLimits
import org.team2471.frc.lib.ctre.d
import org.team2471.frc.lib.ctre.i
import org.team2471.frc.lib.ctre.inverted
import org.team2471.frc.lib.ctre.loggedMotors.LoggedTalonFX
import org.team2471.frc.lib.ctre.magnetSensorOffset
import org.team2471.frc.lib.ctre.motionMagic
import org.team2471.frc.lib.ctre.p
import org.team2471.frc.lib.ctre.remoteCANCoder
import org.team2471.frc.lib.ctre.s
import org.team2471.frc.lib.ctre.setCANCoderAngle
import org.team2471.frc.lib.energy.BatteryLogger
import org.team2471.frc.lib.units.absoluteValue
import org.team2471.frc.lib.units.asFeet
import org.team2471.frc.lib.units.asMeters
import org.team2471.frc.lib.units.asMetersPerSecond
import org.team2471.frc.lib.units.asRadiansPerSecond
import org.team2471.frc.lib.units.asRotation2d
import org.team2471.frc.lib.units.asVolts
import org.team2471.frc.lib.units.cos
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.inches
import org.team2471.frc.lib.units.radians
import org.team2471.frc.lib.units.rotations
import org.team2471.frc.lib.units.rotationsPerSecond
import org.team2471.frc.lib.units.seconds
import org.team2471.frc.lib.units.sin
import org.team2471.frc.lib.units.volts
import org.team2471.frc.lib.units.voltsPerSecond
import org.team2471.frc.lib.util.angleTo
import org.team2471.frc.lib.util.demoMode
import org.team2471.frc.lib.util.isReal
import org.team2471.frc.lib.util.isSim
import kotlin.math.abs
import kotlin.math.cos

object Shooter: SubsystemBase("Shooter") {
    val table = NetworkTableInstance.getDefault().getTable("Shooter")

    // feet, rot/s (of the wheel not the motor) (in an ideal condition. need to divide by SHOOTER_EFFICIENCY)
    val hubSpeedCurve = InterpolatingTreeMap(InverseInterpolator.forDouble(), Interpolator.forDouble()).apply {
        if (isCompBot) {
            put(5.0, 20.0)
            put(7.0, 22.0)
            put(9.0, 22.5)
            put(12.0, 24.7)
            put(15.0, 26.5)
            put(18.0, 29.0)
            put(21.0, 32.0)
        } else {
            put(3.0, 38.5)
            put(6.0, 40.0)
            put(9.0, 46.0)
            put(12.0, 48.0)
            put(15.0, 51.5)
            put(18.0, 58.0)
        }
    }
    // feet, degrees
    val hubAngleCurve = InterpolatingTreeMap(InverseInterpolator.forDouble(), Interpolator.forDouble()).apply {
        if (isCompBot) {
            put(5.0, 75.0)
            put(7.0, 75.0)
            put(9.0, 72.0)
            put(12.0, 64.851)
            put(15.0, 61.085)
            put(18.0, 59.0)
            put(21.0, 57.0)
        } else {
            put(3.0, 81.476)
            put(6.0, 75.118)
            put(9.0, 69.575)
            put(12.0, 64.851)
            put(15.0, 61.085)
            put(18.0, 60.206)
        }
    }

    //feet, s
    val hubTimeCurve = InterpolatingTreeMap(InverseInterpolator.forDouble(), Interpolator.forDouble()).apply {
        if (isCompBot) {
            put(5.0, 0.785)
            put(7.0, 0.992)
            put(9.0, 1.027)
            put(12.0, 1.15)
            put(15.0, 1.2)
            put(18.0, 1.35)
            put(21.0, 1.4)
        } else {
            put(3.0, 1.03)
            put(6.0, 1.08)
            put(9.0, 1.08)
            put(12.0, 1.1)
            put(15.0, 1.2)
            put(18.0, 1.28)
        }
    }

    // feet, rot/s (of the wheel not the motor) (in an ideal condition. need to divide by SHOOTER_EFFICIENCY)
    val floorSpeedCurve = InterpolatingTreeMap(InverseInterpolator.forDouble(), Interpolator.forDouble()).apply {
        if (isCompBot) {
            put(5.0, 25.0)
            put(15.0, 30.988)
            put(25.0, 40.362)
            put(35.0, 70.839)
            put(45.0, 80.88)
        } else {
            put(5.0, 55.0)
            put(10.0, 55.0)
            put(15.0, 55.0)
            put(20.0, 55.0)
        }
    }
    // feet, degrees
    val floorAngleCurve = InterpolatingTreeMap(InverseInterpolator.forDouble(), Interpolator.forDouble()).apply {
        if (isCompBot) {
            put(5.0, 45.0)
            put(15.0, 45.0)
            put(25.0, 45.0)
            put(35.0, 45.0)
            put(45.0, 45.0)
            put(55.0, 45.0)
        } else {
            put(5.0, 45.0)
            put(10.0, 45.0)
            put(15.0, 45.0)
            put(20.0, 45.0)
        }
    }

    // feet, rot/s (of the wheel not the motor) (in an ideal condition. need to divide by SHOOTER_EFFICIENCY)
    val overNetSpeedCurve = InterpolatingTreeMap(InverseInterpolator.forDouble(), Interpolator.forDouble()).apply {
        put(5.0, 30.662)
        put(15.0, 32.198)
        put(25.0, 34.749)
        put(35.0, 38.526)
        put(45.0, 42.613)
        put(55.0, 47.205)
    }
    // feet, degrees
    val overNetAngleCurve = InterpolatingTreeMap(InverseInterpolator.forDouble(), Interpolator.forDouble()).apply {
        put(5.0, 84.72)
        put(15.0, 75.244)
        put(25.0, 66.688)
        put(35.0, 59.223)
        put(45.0, 53.216)
        put(55.0, 48.295)
    }



    val floorTimeCurve = InterpolatingTreeMap(InverseInterpolator.forDouble(), Interpolator.forDouble()).apply {
        if (isCompBot) {
            put(5.0, 0.64)
            put(15.0, 1.03)
            put(25.0, 1.32)
            put(35.0, 1.57)
            put(45.0, 1.8)
            put(55.0, 2.01)
        } else {
            put(5.0, 0.246)
            put(10.0, 0.491)
            put(15.0, 0.737)
            put(20.0, 0.982)
        }
    }

    val overNetTimeCurve = InterpolatingTreeMap(InverseInterpolator.forDouble(), Interpolator.forDouble()).apply {
        put(5.0, 2.0)
        put(15.0, 2.0)
        put(25.0, 2.0)
        put(35.0, 2.0)
        put(45.0, 2.0)
        put(55.0, 2.0)
    }

    val passSpeedCurve: InterpolatingTreeMap<Double, Double> get() = if (!FieldManager.passOverNet) floorSpeedCurve else overNetSpeedCurve
    val passAngleCurve: InterpolatingTreeMap<Double, Double> get() = if (!FieldManager.passOverNet) floorAngleCurve else overNetAngleCurve
    val passTimeCurve: InterpolatingTreeMap<Double, Double> get() = if (!FieldManager.passOverNet) floorTimeCurve else overNetTimeCurve

    val shootingTestSpeedEntry = table.getEntry("Shooter Shooting Speed")
    val shootingTestAngleEntry = table.getEntry("Shooter Shooting Angle")
    val doAutoShootEntry = table.getEntry("Do Auto Shoot")
    val doAutoRampEntry = table.getEntry("Do Auto Ramp Up")

    val zeroHoodButtonEntry = table.getEntry("Zero Hood")

    val demoShootingSpeedEntry = table.getEntry("Demo Shooting Speed")
    val demoShootingAngleEntry = table.getEntry("Demo Shooting Angle")

    val demoShootingSpeed get() = demoShootingSpeedEntry.getDouble(30.0)
    val demoShootingAngle get() = demoShootingAngleEntry.getDouble(65.0)

    val demoAimAtHub get() = OI.driverController.b || OI.driverController.rightStickButton

    val shootingTestSpeed: Double get() = shootingTestSpeedEntry.getDouble(40.0)
    val shootingTestAngle: Double get() = shootingTestAngleEntry.getDouble(40.0)
    val doAutoShoot: Boolean get() = doAutoShootEntry.getBoolean(true) && !(demoMode)
    val doAutoRamp: Boolean get() = doAutoRampEntry.getBoolean(true) && !(demoMode)


    val shooterMotor = LoggedTalonFX(Falcons.SHOOTER_0, CANivores.TURRET_CAN)
    val shooterMotorFollower = LoggedTalonFX(Falcons.SHOOTER_1, CANivores.TURRET_CAN)
    val hoodMotor = LoggedTalonFX(Falcons.SHOOTER_HOOD, CANivores.TURRET_CAN)
    val hoodEncoder = CANcoder(CANCoders.HOOD, CANivores.TURRET_CAN)

    val WHEEL_DIAMETER = 4.0.inches

    val SHOOTER_GEAR_RATIO = if (isCompBot) 1.0 else 18.0/22.0

    // seconds
    const val HOOD_DOWN_TIME = 0.75

    @get:AutoLogOutput(key = "Shooter/Shooter Angular Velocity Setpoint")
    var shooterVelocitySetpoint: AngularVelocity = 0.0.rotationsPerSecond
        set(value) {
            field = value.coerceAtLeast(0.0.rotationsPerSecond)// / SHOOTER_GEAR_RATIO
            if (field > 0.0.rotationsPerSecond) {
                shooterMotor.setControl(MotionMagicVelocityVoltage(field))
            } else {
                if (Robot.isCompBot) {
                    shooterMotor.setControl(NeutralOut())
                } else {
                    shooterMotor.setControl(MotionMagicVoltage(0.0))
                }
            }
        }

    @get:AutoLogOutput(key = "Shooter/Shooter Motor closedLoopReference")
    val shooterMotorReference
        get() = shooterMotor.closedLoopReference.valueAsDouble

    @get:AutoLogOutput(key = "Shooter/ShooterCurve Angular Velocity Setpoint")
    val shooterCurveVelocitySetpoint: AngularVelocity
        get() = shooterVelocitySetpoint * SHOOTER_GEAR_RATIO * shooterEfficiency

    @get:AutoLogOutput(key = "Shooter/Hood Feedforward")
    val hoodFeedforward: Double get() = 0.2//hoodAngle.cos() * 0.2

    // ball trajectory angle
    @get:AutoLogOutput(key = "Shooter/Hood Angle Setpoint")
    var hoodAngleSetpoint: Angle = hoodAngle
        set(value) {
            if (isCompBot) {
                field = value.coerceIn(HOOD_ZERO.degrees, 45.0.degrees)
                hoodMotor.setControl(PositionVoltage(field).withFeedForward(0.0))
            } else {
                field = value.coerceIn(0.0.degrees, 44.0.degrees)
                if (field == 0.0.degrees && hoodAngle > 5.0.degrees) {
                    hoodMotor.setControl(PositionVoltage(field).withFeedForward(hoodFeedforward))
                } else {
                    hoodMotor.setControl(MotionMagicVoltage(field).withFeedForward(hoodFeedforward))
                }
            }

        }

    @get:AutoLogOutput(key = "Shooter/Hood Angle")
    val hoodAngle: Angle get() = hoodMotor.position.valueAsDouble.rotations

    @get:AutoLogOutput(key = "Shooter/Hood Encoder Angle")
    val hoodEncoderAngle: Angle get() = hoodEncoder.position.value

    @get:AutoLogOutput(key = "Shooter/Shooter Angular Velocity")
    val shooterVelocity: AngularVelocity
        get() = shooterMotor.velocity.valueAsDouble.rotationsPerSecond// * SHOOTER_GEAR_RATIO

    val shooterVelocityError: AngularVelocity
        get() = shooterVelocitySetpoint - shooterVelocity

    @get:AutoLogOutput(key = "Shooter/Shooter Current")
    val shooterCurrent: Double get() = shooterMotor.supplyCurrent.valueAsDouble
    @get:AutoLogOutput(key = "Shooter/Shooter Motor Supply Voltage")
    val shooterSupplyVoltage: Double get() = shooterMotor.supplyVoltage.valueAsDouble
    @get:AutoLogOutput(key = "Shooter/Shooter Motor Voltage")
    val shooterMotorVoltage: Double get() = shooterMotor.motorVoltage.valueAsDouble

    @get:AutoLogOutput(key = "Shooter/Hood Current")
    val hoodCurrent: Double get() = hoodMotor.supplyCurrent.valueAsDouble

    // degrees
    const val HOOD_ZERO = 15.0
    val HOOD_UNDER_TRENCH_MAX_ANGLE = if (Robot.isCompBot) 32.0.degrees else 0.0.degrees

    const val BALL_ANGLE_AT_HOOD_ZERO = 90.0

    @get:AutoLogOutput(key = "Shooter/Hood error distance")
    val hoodErrorDistance get() = abs(AimUtils.distanceToTarget.asFeet * sin(hoodMotor.closedLoopError.valueAsDouble.radians))

    @get:AutoLogOutput(key = "Shooter/Velocity error distance")
    val velocityErrorDistance get() = abs((if (AimUtils.isAimingAtGoal) AimUtils.MEASURED_SHOT_AIRTIME * cos(hubAngleCurve.get(AimUtils.distanceToTarget.asFeet)) else AimUtils.PASS_AIRTIME * cos(passAngleCurve.get(AimUtils.distanceToTarget.asFeet))) * shooterMotor.closedLoopError.valueAsDouble * WHEEL_DIAMETER.asMeters * Math.PI * 0.5)

    var fuel: MutableList<FuelSim> = mutableListOf()
    var fuel2: MutableList<FuelSim> = mutableListOf()


    @get:AutoLogOutput(key = "Shooter/raw ramped up")
    val rawRampedUp: Boolean get() = (shooterVelocity - shooterVelocitySetpoint).absoluteValue() < 2.0.rotationsPerSecond

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

        if (!shootingTestSpeedEntry.exists()) shootingTestSpeedEntry.setDouble(shootingTestSpeed)
        shootingTestSpeedEntry.setPersistent()

        if (!shootingTestAngleEntry.exists()) shootingTestAngleEntry.setDouble(shootingTestAngle)
        shootingTestAngleEntry.setPersistent()

        doAutoShootEntry.setBoolean(true)
        doAutoRampEntry.setBoolean(true)

        zeroHoodButtonEntry.setBoolean(false)

        demoShootingSpeedEntry.setDouble(demoShootingSpeed)
        demoShootingAngleEntry.setDouble(demoShootingAngle)

        shooterMotor.applyConfiguration {
            currentLimits(10.0, 30.0, 0.3)
            coastMode()

            Feedback.withSensorToMechanismRatio(1.0/1.5) // Note: I don't think this line configures anything

            inverted(InvertedValue.Clockwise_Positive)

            if (isReal) {
                if (isCompBot) {
                    p(0.4)
                    i(0.4)
                } else {
                    p(0.3)
                    i(0.3)
                }
            } else {
                p(4000.0)
                i(0.0)
            }

            if (isCompBot) {
                MotionMagic.MotionMagicAcceleration = 120.0
            } else {
                MotionMagic.MotionMagicAcceleration = 25.0
            }
        }
        shooterMotor.addFollower(shooterMotorFollower, MotorAlignmentValue.Opposed)

        if (isCompBot) {
            hoodEncoder.applyConfiguration {
                inverted(true)
                magnetSensorOffset(0.046630859)
            }
        }

        hoodMotor.applyConfiguration {
            currentLimits(25.0, 30.0, 1.0)
            inverted(true)
            brakeMode()

            if (isReal) {
                if (isCompBot) {
                    s(0.2, StaticFeedforwardSignValue.UseClosedLoopSign)
                    p(200.0)
                    d(0.0)
                } else {
                    s(0.05, StaticFeedforwardSignValue.UseClosedLoopSign)
                    p(60.0)
                    d(0.0)
                }
            } else {
                s(0.05, StaticFeedforwardSignValue.UseClosedLoopSign)
                p(60.0)
                d(4.0)
            }

            if (!isCompBot) {
                motionMagic(0.75, 5.0)
            }
            remoteCANCoder(hoodEncoder.deviceID, if (Robot.isCompBot) 85.5 else 9.64285714285714)
        }
    }



    override fun periodic() {
        LoopLogger.record("b4 Shooter periodic")
        if (isSim) {
            if (isShooting) {
                if (i > 3) {
                    i = 0
                    shootSimulatedFuel()
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

        if (zeroHoodButtonEntry.getBoolean(false)) {
            hoodEncoder.setCANCoderAngle(HOOD_ZERO.degrees)
            zeroHoodButtonEntry.setBoolean(false)
            println("Zeroed hood")
        }

        BatteryLogger.recordCurrent("Shooter Roller", shooterMotor.supplyCurrent.value * 2.0)
        BatteryLogger.recordCurrent("Hood", hoodMotor.supplyCurrent.value)

        LoopLogger.record("Shooter periodic")
    }

    fun default(): Command = runCommand(this) {
        if ((doAutoShoot && !Drive.cameraDisconnected) && Drive.useAprilTags && AimUtils.isAimingAtGoal && !demoMode) {
            if (FieldManager.inScoringZone && !FieldManager.inNoShootArea /*&& AimUtils.distanceToTarget < 13.0.feet*/ && FieldManager.shouldShoot) {
                shootLoop()
            } else {
                isShooting = false
                if (Intake.intakeState != Intake.IntakeState.INTAKING) {
                    Spindexer.currentState = Spindexer.State.OFF
                }
                hoodAngleSetpoint =
                    hoodAngleSetpoint.coerceAtMost(HOOD_UNDER_TRENCH_MAX_ANGLE)//HOOD_STOW_SETPOINT.degrees
            }
            if (FieldManager.shouldShoot && FieldManager.inScoringZone) {
                rampUpLoop()
            }
        } else {
            hoodAngleSetpoint =
                hoodAngleSetpoint.coerceAtMost(HOOD_UNDER_TRENCH_MAX_ANGLE)//HOOD_STOW_SETPOINT.degrees
            shooterVelocitySetpoint =
                0.0.rotationsPerSecond//if (doAutoRamp) 50.0.rotationsPerSecond / SHOOTER_GEAR_RATIO / AimUtils.shooterEfficiency else 0.0.rotationsPerSecond
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
            }.onlyRunWhileTrue { OI.driverController.rightTriggerAxis >= 0.1 || OI.driverController.rightStickButton }.repeatedly(),
            runCommand {
                isShooting = false
                Spindexer.currentState = Spindexer.State.OFF
                hoodAngleSetpoint = hoodAngleSetpoint.coerceAtMost(HOOD_UNDER_TRENCH_MAX_ANGLE)//HOOD_STOW_SETPOINT.degrees
            }.onlyRunWhileFalse { OI.driverController.rightTriggerAxis >= 0.1 || OI.driverController.rightStickButton }.repeatedly()
        ).finallyRun {
            isShooting = false
            Spindexer.currentState = Spindexer.State.OFF
            hoodAngleSetpoint = hoodAngleSetpoint.coerceAtMost(HOOD_UNDER_TRENCH_MAX_ANGLE)//HOOD_STOW_SETPOINT.degrees
        }
    }


    fun shoot(ignoreRampUp: Boolean = false): Command = runCommand(Shooter) {
        shootLoop(ignoreRampUp)
        rampUpLoop()
    }.finallyRun {
        isShooting = false
        Spindexer.currentState = Spindexer.State.OFF
        hoodAngleSetpoint = HOOD_ZERO.degrees
    }

    fun rampUpLoop() {
        shooterVelocitySetpoint = if (!demoMode || demoAimAtHub) AimUtils.getShooterRPS() else demoShootingSpeed.rotationsPerSecond
    }

    fun shootLoop(ignoreRampUp: Boolean = false) {
//        println("Shoot Loop!!!")
        if ((!FieldManager.inNoShootArea || ignoreRampUp) && (!Turret.isTurretWrapping || Turret.disableTurret) && (((rampedUp || ignoreRampUp) && AimUtils.isAimingAtGoal) || (rampedUpPassing && !AimUtils.isAimingAtGoal)) && (FieldManager.shouldShoot || !AimUtils.isAimingAtGoal)) {
            isShooting = true
            Spindexer.currentState = Spindexer.State.ON
        } else {
            isShooting = false
            Spindexer.currentState = Spindexer.State.OFF
        }

        val wantedHoodSetpoint = (
            if (Turret.isTurretWrapping)
                HOOD_ZERO
            else
                if (!demoMode || demoAimAtHub)
                    if (AimUtils.isAimingAtGoal || demoMode)
                        BALL_ANGLE_AT_HOOD_ZERO - hubAngleCurve.get(AimUtils.distanceToTarget.asFeet)
                    else
                        BALL_ANGLE_AT_HOOD_ZERO - passAngleCurve.get(AimUtils.distanceToTarget.asFeet)
                else
                    BALL_ANGLE_AT_HOOD_ZERO - demoShootingAngle
            ).degrees


        if (FieldManager.inNoShootArea) {
            hoodAngleSetpoint = wantedHoodSetpoint.coerceAtMost(HOOD_UNDER_TRENCH_MAX_ANGLE)
        } else {
            hoodAngleSetpoint = wantedHoodSetpoint
        }

    }


    fun shootSimulatedFuel() {
        val exitVelocity = (AimUtils.getShooterRPS() * SHOOTER_GEAR_RATIO * AimUtils.shooterEfficiency).toExitVelocity().asMetersPerSecond
        val exitAngle = if (AimUtils.isAimingAtGoal) hubAngleCurve.get(AimUtils.distanceToTarget.asFeet).degrees else passAngleCurve.get(AimUtils.distanceToTarget.asFeet).degrees
        val angleToTarget = Turret.turretTranslation.angleTo(AimUtils.aimTarget)
        val velocity2d = Translation2d(exitVelocity * exitAngle.cos(), 0.0).rotateBy(angleToTarget.asRotation2d)
        val turretVelocity = Translation2d(Turret.turretOffsetFromCenter.x * Drive.gyroYawRate.asRadiansPerSecond, Turret.turretOffsetFromCenter.y * Drive.gyroYawRate.asRadiansPerSecond).rotateBy(Drive.heading) + Drive.velocity
        fuel.add(FuelSim(
            Translation3d(Turret.turretTranslation.x, Turret.turretTranslation.y, 0.4),
            Translation3d(velocity2d.x + turretVelocity.x, velocity2d.y + turretVelocity.y, exitVelocity * exitAngle.sin())
        ))
    }

    fun shootSimulatedFuelWithMotors() {
        val exitVelocity = (shooterVelocity * SHOOTER_GEAR_RATIO * AimUtils.shooterEfficiency).toExitVelocity().asMetersPerSecond
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