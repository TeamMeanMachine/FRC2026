@file:Suppress("unused")

package frc.team2471.frc2026

import com.ctre.phoenix6.CANBus

object Sparks {
}

object AnalogSensors {
}

object DigitalSensors {
    const val INTAKE_STOP_SENSOR_0 = 9
    const val INTAKE_STOP_SENSOR_1 = 1
}

object Falcons {
    const val FRONT_RIGHT_DRIVE = 23
    const val FRONT_RIGHT_STEER = 22

    const val FRONT_LEFT_DRIVE = 16
    const val FRONT_LEFT_STEER = 15

    const val BACK_RIGHT_DRIVE = 1
    const val BACK_RIGHT_STEER = 24

    const val BACK_LEFT_DRIVE = 10
    const val BACK_LEFT_STEER = 11



    const val INTAKE_DEPLOY_0 = 21
    const val INTAKE_DEPLOY_1 = 20
    const val INTAKE_ROLLER_0 = 14
    const val INTAKE_ROLLER_1 = 13

    const val SHOOTER_HOOD = 6

    // if electrical problem, add 1 (they started at 1 smh)
    const val SHOOTER_0 = 4
    const val SHOOTER_1 = 5

    const val TURRET_0 = 7
    const val TURRET_1 = 8

    const val SPIN_0 = 2
    const val SPIN_1 = 3
    const val SIDETAKE = 19
    const val UPTAKE = 18
}

object Talons {
}

object CANCoders {
    const val FRONT_RIGHT = 31
    const val FRONT_LEFT = 30
    const val BACK_RIGHT = 32
    const val BACK_LEFT = 33

    const val TURRET_0 = 35
    const val TURRET_1 = 36

    const val HOOD = 37
}

object PWMPort {
}

object ServoPort {
}

object CANivores {
    val TURRET_CAN = CANBus(if (Robot.isCompBot) "Ken A Vore" else "kenivore")
    val INTAKE_CAN = CANBus("Intake A Vore")
}

object CANSensors {
    const val PIGEON = 34
    const val TURRET_PIGEON = 37
}

object I2CPort {
}