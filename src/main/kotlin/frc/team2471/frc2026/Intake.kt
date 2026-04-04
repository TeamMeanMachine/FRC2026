package frc.team2471.frc2026

import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.NeutralOut
import com.ctre.phoenix6.controls.TorqueCurrentFOC
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team2471.frc2026.Robot.powerTracker
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.littletonrobotics.junction.AutoLogOutput
import org.team2471.frc.lib.control.CurrentLimits
import org.team2471.frc.lib.control.LoopLogger
import org.team2471.frc.lib.control.commands.finallyRun
import org.team2471.frc.lib.control.commands.onlyRunWhileFalse
import org.team2471.frc.lib.control.commands.onlyRunWhileTrue
import org.team2471.frc.lib.control.commands.parallelCommand
import org.team2471.frc.lib.control.commands.runCommand
import org.team2471.frc.lib.control.commands.runOnceCommand
import org.team2471.frc.lib.control.commands.sequenceCommand
import org.team2471.frc.lib.control.commands.waitCommand
import org.team2471.frc.lib.ctre.addFollower
import org.team2471.frc.lib.ctre.applyConfiguration
import org.team2471.frc.lib.ctre.coastMode
import org.team2471.frc.lib.ctre.currentLimits
import org.team2471.frc.lib.ctre.motionMagic
import org.team2471.frc.lib.ctre.p
import org.team2471.frc.lib.ctre.s
import org.team2471.frc.lib.units.asVolts
import org.team2471.frc.lib.util.isSim
import kotlin.math.absoluteValue

object Intake: SubsystemBase("Intake") {
    private val table = NetworkTableInstance.getDefault().getTable("Intake")

    val deployPoseEntry = table.getEntry("deployPose")
    val stowPoseEntry = table.getEntry("stowPose")
    val deepStowPoseEntry = table.getEntry("deepStowPose")
    val intakePowerEntry = table.getEntry("intakePower")

    val DEPLOY_POSE get() = deployPoseEntry.getDouble(25.75)
    val STOW_POSE get() = stowPoseEntry.getDouble(2.0)
    val DEEP_STOW_POSE get() = deepStowPoseEntry.getDouble(0.0)

    val INTAKE_POWER get() = intakePowerEntry.getDouble(75.0)
    const val HOMING_POWER = 0.15

    val rollerMotor = TalonFX(Falcons.INTAKE_ROLLER_0)
    val rollerMotorFollower = TalonFX(Falcons.INTAKE_ROLLER_1)
    val deployMotor0 = TalonFX(Falcons.INTAKE_DEPLOY_0)
    val deployMotor1 = TalonFX(Falcons.INTAKE_DEPLOY_1)
    val stopSensor0 = DigitalInput(DigitalSensors.INTAKE_STOP_SENSOR_0)
    val stopSensor1 = DigitalInput(DigitalSensors.INTAKE_STOP_SENSOR_1)

    @get:AutoLogOutput(key = "Intake/Intake state")
    var intakeState: IntakeState = IntakeState.OFF
    var prevIntakeState = intakeState

    @get:AutoLogOutput(key = "Intake/Hit Hard Stop 0")
    val hitHardStop0 get() = !stopSensor0.get()

    @get:AutoLogOutput(key = "Intake/Hit Hard Stop 1")
    val hitHardStop1 get() = !stopSensor1.get()

    @get:AutoLogOutput(key = "Intake/Roller Motor Temp")
    val rollerTemp get() = rollerMotor.deviceTemp.valueAsDouble

    @get:AutoLogOutput(key = "Intake/Velocity Setpoint")
    var velocitySetpoint: Double = 0.0
        set(value) {
            field = value.coerceIn(-100.0, 100.0)
            if (field == 0.0) {
                rollerMotor.setControl(NeutralOut())
            } else {
                rollerMotor.setControl(DutyCycleOut(field / 100.0).withEnableFOC(true))
            }
        }

    var lastReachedSetpoint = 0.0

    @get:AutoLogOutput(key = "Intake/Deploy Setpoint")
    var deploySetpoint: Double = 0.0
        set(value) {
            field = value
            if (finishedHoming) {
                if (lastReachedSetpoint != value) {
                    reachedSetpoint = false
                    lastReachedSetpoint = value
                }

                if (Robot.isCompBot) {
                    if (deployMotor0Error.absoluteValue < 0.5 && deployMotor1Error.absoluteValue < 0.5) {
                        reachedSetpoint = true
                    }
                } else {
                    if (deployMotor0Error.absoluteValue < 0.5) {
                        reachedSetpoint = true
                    }
                }


                if (reachedSetpoint) {
                    // Relies on the optimization thing where it won't evaluate the statements after the or if the first statement returns true, to prevent accessing deployMotor1 stuff when it doesn't exist
                    if (!Robot.isCompBot || (deployMotor0Position - deployMotor1Position).absoluteValue < FLEX_THRESHOLD) {
                        if (deployMotor0Error < -0.7) {
                            deployMotor0.setControl(TorqueCurrentFOC(21.0))
                        } else {
                            deployMotor0.setControl(NeutralOut())
                        }

                        if (Robot.isCompBot) {
                            if (deployMotor1Error < -0.7) {
                                deployMotor1.setControl(TorqueCurrentFOC(21.0))
                            } else {
                                deployMotor1.setControl(NeutralOut())
                            }
                        }
                    } else {
                        if (deployMotor0Position > deployMotor1Position) {
                            deployMotor0.setControl(MotionMagicVoltage(deployMotor1Position))
                            deployMotor1.setControl(TorqueCurrentFOC(21.0))
                        } else {
                            deployMotor1.setControl(MotionMagicVoltage(deployMotor0Position))
                            deployMotor0.setControl(TorqueCurrentFOC(21.0))
                        }
                    }
                } else {
                    deployMotor0.setControl(MotionMagicVoltage(field))
                    if (Robot.isCompBot) {
                        deployMotor1.setControl(MotionMagicVoltage(field))
                    }
                }
            }
        }

    @get:AutoLogOutput(key = "Intake/Roller Velocity")
    val rollerVelocity: Double
        get() = rollerMotor.velocity.valueAsDouble

    @get:AutoLogOutput(key = "Intake/Roller Current")
    val rollerCurrent: Double
        get() = rollerMotor.supplyCurrent.valueAsDouble

//    @get:AutoLogOutput(key = "Intake/Deploy Current")
//    val deployCurrent: Double
//        get() = deployMotor0.supplyCurrent.valueAsDouble

    @get:AutoLogOutput(key = "Intake/Deploy Motor Position")
    val deployMotor0Position: Double
        get() = deployMotor0.position.valueAsDouble

    @get:AutoLogOutput(key = "Intake/Deploy Motor Follower Position")
    val deployMotor1Position: Double
        get() = if (Robot.isCompBot) deployMotor1.position.valueAsDouble else 0.0


    @get:AutoLogOutput(key = "Intake/Deploy Motor Error")
    val deployMotor0Error: Double
        get() = deployMotor0Position - deploySetpoint//deployMotor.closedLoopError.valueAsDouble

    @get:AutoLogOutput(key = "Intake/Deploy Motor Follower Error")
    val deployMotor1Error: Double
        get() = deployMotor1Position - deploySetpoint//deployMotor.closedLoopError.valueAsDouble


    var finishedHoming: Boolean = false
    var isDeployed: Boolean = false

    @get:AutoLogOutput(key = "Intake/goingToSetpoint0")
    var goingToSetpoint0: Boolean = false

    @get:AutoLogOutput(key = "Intake/goingToSetpoint1")
    var goingToSetpoint1: Boolean = false

    @get:AutoLogOutput(key = "Intake/reachedSetpoint0")
    var reachedSetpoint: Boolean = false


    const val FLEX_THRESHOLD = 3.0

    val autoCurrentLimits = CurrentLimits(30.0, 40.0, 1.0)
    val teleopCurrentLimits = CurrentLimits(13.0, 40.0, 0.2)


    init {
        if (!deployPoseEntry.exists()) deployPoseEntry.setDouble(DEPLOY_POSE)
        if (!stowPoseEntry.exists()) stowPoseEntry.setDouble(STOW_POSE)
        if (!deepStowPoseEntry.exists()) deepStowPoseEntry.setDouble(DEEP_STOW_POSE)
        if (!intakePowerEntry.exists()) intakePowerEntry.setDouble(INTAKE_POWER)

        deployPoseEntry.setPersistent()
        stowPoseEntry.setPersistent()
        deepStowPoseEntry.setPersistent()
        intakePowerEntry.setPersistent()



        deployMotor0.applyConfiguration {
            currentLimits(5.0, 25.0, 0.25)
            coastMode()
            p(1.5)
            s(0.25, StaticFeedforwardSignValue.UseClosedLoopSign)

            p(0.05, 1)
            s(0.25, StaticFeedforwardSignValue.UseClosedLoopSign, 1)

            motionMagic(750.0, 1500.0)
        }
        deployMotor0.setPosition(0.0)

        if (Robot.isCompBot) {
            deployMotor1.applyConfiguration {
                currentLimits(5.0, 25.0, 0.25)
                coastMode()
                p(1.5)
                s(0.25, StaticFeedforwardSignValue.UseClosedLoopSign)

                p(0.05, 1)
                s(0.25, StaticFeedforwardSignValue.UseClosedLoopSign, 1)

                motionMagic(750.0, 1500.0)
            }
            deployMotor1.setPosition(0.0)
        }

        rollerMotor.applyConfiguration {
            currentLimits(autoCurrentLimits.peakLimit, autoCurrentLimits.continuousLimit, autoCurrentLimits.peakDuration)
            p(7.0)
            s(10.0, StaticFeedforwardSignValue.UseVelocitySign)
            coastMode()
        }
        rollerMotor.addFollower(rollerMotorFollower)

        if (!isSim) {
            powerTracker.addMotors("Intake Roller", { rollerMotor.supplyCurrent.valueAsDouble }, 2, {rollerMotor.supplyVoltage.value.asVolts})
            powerTracker.addMotors("Intake Deploy 0", { deployMotor0.supplyCurrent.valueAsDouble })
            if (Robot.isCompBot) {
                powerTracker.addMotors("Intake Deploy 1", { deployMotor1.supplyCurrent.valueAsDouble })
            }
        }

        this.defaultCommand = default()


        GlobalScope.launch {
            org.team2471.frc.lib.coroutines.periodic {
                deploySetpoint = deploySetpoint
            }
        }
    }

    fun deploy() {
        if (deployMotor0Error.absoluteValue > FLEX_THRESHOLD) {
            goingToSetpoint0 = true
        }
        if (Robot.isCompBot && deployMotor1Error.absoluteValue > FLEX_THRESHOLD) {
            goingToSetpoint1 = true
        }
        deploySetpoint = DEPLOY_POSE
        isDeployed = true
    }

    fun stow() {
        goingToSetpoint0 = true
        if (Robot.isCompBot) goingToSetpoint1 = true
        deploySetpoint = STOW_POSE
        isDeployed = false
    }

    fun deepStow() {
        goingToSetpoint0 = true
        if (Robot.isCompBot) goingToSetpoint1 = true
        deploySetpoint = DEEP_STOW_POSE
        isDeployed = false
    }


    fun home(): Command  = if (Robot.isCompBot) {
        parallelCommand(
            runOnceCommand {
                finishedHoming = false
            },
            homeMotor(deployMotor0, { hitHardStop0 }),
            homeMotor(deployMotor1, { hitHardStop1 })
        ).finallyRun {
            finishedHoming = true
            stow()
        }
    } else {
        sequenceCommand(
            runOnceCommand {
                finishedHoming = false
            },
            homeMotor(deployMotor0, { hitHardStop0 }),
            runOnceCommand {
                finishedHoming = true
                stow()
            }
        )
    }.apply {
        addRequirements(Intake, Shooter)
    }

    private fun homeMotor(motor: TalonFX, hitHardStopSupplier: () -> Boolean): Command {
        return sequenceCommand(
            runCommand {
                println("going in?")
                motor.setControl(DutyCycleOut(HOMING_POWER))
            }.onlyRunWhileTrue { hitHardStopSupplier.invoke() },
            runCommand {
                println("going out?")
                motor.setControl(DutyCycleOut(-HOMING_POWER))
            }.onlyRunWhileFalse { hitHardStopSupplier.invoke() }.withTimeout(6.0).finallyRun {
                motor.setControl(DutyCycleOut(0.0))
                println("Deploy Pos: ${motor.position}")
                motor.setPosition(0.13)
            }
        )
    }

    fun pulse(): Command = sequenceCommand(
        runOnceCommand { stow() },
        waitCommand(0.25),
        runOnceCommand { deploy() },
        waitCommand(0.25),
    ).repeatedly().onlyRunWhileTrue { Robot.isAutonomous || OI.driverController.rightTriggerAxis > 0.75 }.finallyRun {
        stow()
    }


    fun homeDeploy(): Command = runOnce {
        deployMotor0.setPosition(deploySetpoint)
        if (Robot.isCompBot) deployMotor1.setPosition(deploySetpoint)
    }


    private fun default(): Command = run {
        LoopLogger.record("b4 Intake default")
        when (intakeState) {
            IntakeState.OFF -> {
                velocitySetpoint = 0.0
            }
            IntakeState.INTAKING -> {
                velocitySetpoint = if (Robot.isAutonomous) 100.0 else INTAKE_POWER
                if (!Shooter.isShooting) {
                    Spindexer.currentState = Spindexer.State.AGITATING
                }
            }
            IntakeState.SPITTING -> {
                velocitySetpoint = -INTAKE_POWER
            }
        }

        LoopLogger.record("Intake default when")

        if (prevIntakeState == IntakeState.INTAKING && intakeState != IntakeState.INTAKING) {
            Spindexer.currentState = Spindexer.State.OFF
        }
        prevIntakeState = intakeState

        if (goingToSetpoint0 && deployMotor0Error.absoluteValue < FLEX_THRESHOLD) {
            goingToSetpoint0 = false
        }

        if (Robot.isCompBot) {
            if (goingToSetpoint1 && deployMotor1Error.absoluteValue < FLEX_THRESHOLD) {
                goingToSetpoint1 = false
            }
        }


        LoopLogger.record("Intake default b4 controlMode")


        LoopLogger.record("Intake default")
    }

    enum class IntakeState {
        INTAKING,
        OFF,
        SPITTING,
    }
}