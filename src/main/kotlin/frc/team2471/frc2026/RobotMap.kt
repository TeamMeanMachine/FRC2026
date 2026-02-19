@file:Suppress("unused")

package frc.team2471.frc2026

import com.ctre.phoenix6.CANBus

object Sparks {
}

object AnalogSensors {
}

object DigitalSensors {
    const val INTAKE_STOP_SENSOR = 0
}

object Falcons {
    const val FRONT_RIGHT_DRIVE = 23
    const val FRONT_RIGHT_STEER = 22

    const val FRONT_LEFT_DRIVE = 16
    const val FRONT_LEFT_STEER = 15

    const val BACK_RIGHT_DRIVE = 1
    val BACK_RIGHT_STEER = 24

    const val BACK_LEFT_DRIVE = 10
    const val BACK_LEFT_STEER = 11



    const val INTAKE_DEPLOY = 21
    const val INTAKE_ROLLER_0 = 14
    const val INTAKE_ROLLER_1 = 13

    const val SHOOTER_HOOD = 53
    const val SHOOTER_0 = 57
    const val SHOOTER_1 = 56

    const val TURRET_0 = 55
    const val TURRET_1 = 54

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

    const val TURRET_1 = 99
    const val TURRET_2 = 98
}

object PWMPort {
}

object ServoPort {
}

object CANivores {
    val TURRET_CAN = CANBus("kenivore")
}

object CANSensors {
    const val PIGEON = 34
    const val TURRET_PIGEON = 35
}

object I2CPort {
}