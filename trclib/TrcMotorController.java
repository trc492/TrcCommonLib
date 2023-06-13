/*
 * Copyright (c) 2023 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package TrcCommonLib.trclib;

/**
 * This interface implements a generic motor. Some motors natively support some of the methods but simulate others.
 * Some methods may be unsupported by the motor in which case it may throw an UnsupportedOperationException.
 */
public interface TrcMotorController
{
    /**
     * This method resets the motor controller configurations to factory default so that everything is at known state.
     */
    void resetFactoryDefault();

    /**
     * This method returns the bus voltage of the motor controller.
     *
     * @return bus voltage of the motor controller.
     */
    double getBusVoltage();

    /**
     * This method checks if the motor controller is connected to the robot. Note that this does NOT guarantee the
     * connection status of the motor to the motor controller. If detecting the motor presence is impossible (i.e. the
     * motor controller is connected via PWM) this method will always return true.
     *
     * @return true if the motor is connected or if it's impossible to know, false otherwise.
     */
    boolean isConnected();

    /**
     * This method is used to check if the motor controller supports close loop control natively. Motor subclass
     * should override this to report true if it supports close loop control natively. TrcMotor default is set to
     * false.
     *
     * @return true if motor controller supports close loop control, false otherwise.
     */
    boolean supportCloseLoopControl();

    /**
     * This method enables voltage compensation so that it will maintain the motor output regardless of battery
     * voltage.
     *
     * @param batteryNominalVoltage specifies the nominal voltage of the battery.
     */
    void enableVoltageCompensation(double batteryNominalVoltage);

    /**
     * This method disables voltage compensation.
     */
    void disableVoltageCompensation();

    /**
     * This method checks if voltage compensation is enabled.
     *
     * @return true if voltage compensation is enabled, false if disabled.
     */
    boolean isVoltageCompensationEnabled();

    /**
     * This method enables/disables motor brake mode. In motor brake mode, set power to 0 would stop the motor very
     * abruptly by shorting the motor wires together using the generated back EMF to stop the motor. When not enabled,
     * (i.e. float/coast mode), the motor wires are just disconnected from the motor controller so the motor will
     * stop gradually.
     *
     * @param enabled specifies true to enable brake mode, false otherwise.
     */
    void setBrakeModeEnabled(boolean enabled);

    /**
     * This method inverts the spinning direction of the motor.
     *
     * @param inverted specifies true to invert motor direction, false otherwise.
     */
    void setMotorInverted(boolean inverted);

    /**
     * This method checks if the motor direction is inverted.
     *
     * @return true if motor direction is inverted, false otherwise.
     */
    boolean isMotorInverted();

    /**
     * This method stops the motor regardless of what control mode the motor is on.
     */
    void stopMotor();

    /**
     * This method sets the percentage motor power.
     *
     * @param power specifies the percentage power (range -1.0 to 1.0) to be set.
     */
    void setMotorPower(double power);

    /**
     * This method gets the current motor power.
     *
     * @return current motor power.
     */
    double getMotorPower();

    /**
     * This method commands the motor to spin at the given current value using close loop control.
     *
     * @param current specifies current in amperes.
     */
    void setMotorCurrent(double current);

    /**
     * This method returns the motor current.
     *
     * @return motor current in amperes.
     */
    double getMotorCurrent();

    /**
     * This method sets the current limit of the motor.
     *
     * @param currentLimit specifies the current limit (holding current) in amperes when feature is activated.
     * @param triggerThresholdCurrent specifies threshold current in amperes to be exceeded before limiting occurs.
     *        If this value is less than currentLimit, then currentLimit is used as the threshold.
     * @param triggerThresholdTime specifies how long current must exceed threshold (seconds) before limiting occurs.
     */
    void setCurrentLimit(double currentLimit, double triggerThresholdCurrent, double triggerThresholdTime);

    /**
     * This method inverts the active state of the reverse limit switch, typically reflecting whether the switch is
     * wired normally open or normally close.
     *
     * @param inverted specifies true to invert the limit switch, false otherwise.
     */
    void setRevLimitSwitchInverted(boolean inverted);

    /**
     * This method inverts the active state of the forward limit switch, typically reflecting whether the switch is
     * wired normally open or normally close.
     *
     * @param inverted specifies true to invert the limit switch, false otherwise.
     */
    void setFwdLimitSwitchInverted(boolean inverted);

    /**
     * This method returns the state of the reverse limit switch.
     *
     * @return true if reverse limit switch is active, false otherwise.
     */
    boolean isRevLimitSwitchActive();

    /**
     * This method returns the state of the forward limit switch.
     *
     * @return true if forward limit switch is active, false otherwise.
     */
    boolean isFwdLimitSwitchActive();

    /**
     * This method inverts the position sensor direction. This may be rare but there are scenarios where the motor
     * encoder may be mounted somewhere in the power train that it rotates opposite to the motor rotation. This will
     * cause the encoder reading to go down when the motor is receiving positive power. This method can correct this
     * situation.
     *
     * @param inverted specifies true to invert position sensor direction, false otherwise.
     */
    void setPositionSensorInverted(boolean inverted);

    /**
     * This method returns the state of the position sensor direction.
     *
     * @return true if the motor direction is inverted, false otherwise.
     */
    boolean isPositionSensorInverted();

    /**
     * This method returns the motor position by reading the position sensor. The position sensor can be an encoder
     * or a potentiometer.
     *
     * @return current motor position in raw sensor units.
     */
    double getMotorPosition();

    /**
     * This method resets the motor position sensor, typically an encoder.
     */
    void resetMotorPosition();

    /**
     * This method returns the current motor velocity.
     *
     * @return current motor velocity in raw sensor units per sec.
     */
    double getMotorVelocity();

    /**
     * This method sets the close loop percentage output limits. By default the limits are set to the max at -1 to 1.
     * By setting a non-default limits, it effectively limits the output power of the close loop control.
     *
     * @param revLimit specifies the percentage output limit of the reverse direction.
     * @param fwdLimit specifies the percentage output limit of the forward direction.
     */
    void setCloseLoopOutputLimits(double revLimit, double fwdLimit);

    //
    // The following methods are provided by TrcMotor to simulate some motor features. If the motor sub-class can
    // support these features, it should override the these TrcMotor methods.
    //

    /**
     * This method sets this motor to follow another motor. Motor subclass should override this if it supports this
     * natively. TrcMotor by default supports software follower list.
     *
     * @param motor specifies the motor to follow.
     */
    void followMotor(TrcMotor motor);

    /**
     * This method sets the PID coefficients of the motor's Position PID controller.
     *
     * @param pidCoeff specifies the PID coefficients to set.
     */
    void setPositionPidCoefficients(TrcPidController.PidCoefficients pidCoeff);

    /**
     * This method returns the PID coefficients of the motor's Position PID controller.
     *
     * @return PID coefficients of the motor's position PID controller.
     */
    TrcPidController.PidCoefficients getPositionPidCoefficients();

    /**
     * This method sets the PID coefficients of the motor's velocity PID controller.
     *
     * @param pidCoeff specifies the PID coefficients to set.
     */
    void setVelocityPidCoefficients(TrcPidController.PidCoefficients pidCoeff);

    /**
     * This method returns the PID coefficients of the motor's velocity PID controller.
     *
     * @return PID coefficients of the motor's veloicty PID controller.
     */
    TrcPidController.PidCoefficients getVelocityPidCoefficients();

    /**
     * This method sets the motor controller to position control mode.
     *
     * @param pidCoeff specifies the PID coefficients for position PID control.
     * @param useSoftwarePid specifies true to use software PID control, false to use motor built-in PID.
     */
    void enablePositionMode(TrcPidController.PidCoefficients pidCoeff, boolean useSoftwarePid);

    /**
     * This method disables position control mode returning it to power control mode.
     */
    void disablePositionMode();

    /**
     * This method sets the motor controller to velocity control mode.
     *
     * @param pidCoeff specifies the PID coefficients for velocity PID control.
     * @param useSoftwarePid specifies true to use software PID control, false to use motor built-in PID.
     */
    void enableVelocityMode(TrcPidController.PidCoefficients pidCoeff, boolean useSoftwarePid);

    /**
     * This method disables velocity control mode returning it to power control mode.
     */
    void disableVelocityMode();

    /**
     * This method returns the current control mode.
     *
     * @return current control mode.
     */
    TrcMotor.ControlMode getControlMode();

    /**
     * This method commands the motor to go to the given position using close loop control.
     *
     * @param pos specifies the motor position in raw sensor units.
     */
    void setMotorPosition(double pos);

    /**
     * This method commands the motor to spin at the given velocity using close loop control.
     *
     * @param vel specifies the motor velocity in raw sensor units per second.
     */
    void setMotorVelocity(double vel);

}   //interface TrcMotorController
