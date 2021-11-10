package org.firstinspires.ftc.teamcode.a_opmodes.teleop


import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B
import org.firstinspires.ftc.teamcode.a_opmodes.teleop.MainTeleOp.TemplateState.*


class StateMap {
    public var stateMap = mapOf(
            INTAKE to mapOf(
                    A to TRANSPORT,
                    B to OUTTAKE
            ),
            TRANSPORT to mapOf(
                    A to INTAKE
            )
    )

}