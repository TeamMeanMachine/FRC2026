@file:Suppress("unused")

package frc.team2471.frc2026

import org.team2471.frc.lib.util.isSim

object Sparks {
}

object AnalogSensors {
}

object DigitalSensors {
}

object Falcons {
    const val FRONT_RIGHT_DRIVE = 23
    const val FRONT_RIGHT_STEER = 14

    const val FRONT_LEFT_DRIVE = 16
    const val FRONT_LEFT_STEER = 10

    const val BACK_RIGHT_DRIVE = 22
    val BACK_RIGHT_STEER = if (isSim) 40 else 15

    val BACK_LEFT_DRIVE = if (isSim) 41 else 18
    const val BACK_LEFT_STEER = 11

}

object Talons {
}

object CANCoders {
    const val FRONT_RIGHT = 25
    const val FRONT_LEFT = 27
    const val BACK_RIGHT = 26
    const val BACK_LEFT = 28
}

object PWMPort {
}

object ServoPort {
}

object CANivores {
}

object CANSensors {
    const val PIGEON = 2
}

object I2CPort {
}