/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.core238.wrappers;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.buttons.Button;

public class AxisButton extends Button {

    private final BooleanSupplier isPressed;

    public AxisButton(final XboxController joystick, final Hand hand, final Axis axis, final double threshhold) {
        isPressed = getIsPressedFunction(joystick, hand, axis, threshhold);
    }

    @Override
    public boolean get() {
        return isPressed.getAsBoolean();
    }

    private BooleanSupplier getIsPressedFunction(final GenericHID joystick, final Hand hand, final Axis axis,
            final double threshhold) {
        switch (axis) {
        case X:
            return threshhold > 0 
                ? () -> joystick.getX(hand) >= threshhold 
                : () -> joystick.getX(hand) <= threshhold;
        case Y:
        default:
            return threshhold > 0 
                ? () -> joystick.getY(hand) >= threshhold 
                : () -> joystick.getY(hand) <= threshhold;
        }
    }

    public enum Axis {
        X(1), Y(2);

        public final int value;

        Axis(final int value) {
            this.value = value;
        }
    }

}
