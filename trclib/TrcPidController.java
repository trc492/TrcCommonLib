/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
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

import java.util.EmptyStackException;
import java.util.Locale;
import java.util.Stack;

/**
 * This class implements a PID controller. A PID controller takes a target set point and an input from a feedback
 * device to calculate the output power of an effector usually a motor or a set of motors.
 */
public class TrcPidController
{
    private static final TrcDbgTrace globalTracer = TrcDbgTrace.getGlobalTracer();
    private static final boolean debugEnabled = false;

    public static final double DEF_SETTLING_TIME = 0.2;

    /**
     * Some actuators are non-linear. The load may vary depending on the position. For example, raising an arm
     * against gravity will have the maximum load when the arm is horizontal and zero load when vertical. This
     * caused problem when applying PID control on this kind of actuator because PID controller is only good at
     * controlling linear actuators. To make PID controller works for non-linear actuators, we need to add power
     * compensation that counteracts the non-linear component of the load so that PID only deals with the resulting
     * linear load. However, a generic PID controller doesn't understand the actuator and has no way to come up
     * with the compensation. Therefore, it is up to the user of the TrcPIDMotor to provide this interface for
     * computing the output compensation.
     */
    public interface PowerCompensation
    {
        /**
         * This method is called to compute the power compensation to counteract the varying non-linear load.
         *
         * @param currPower specifies the current motor power.
         * @return compensation value of the actuator.
         */
        double getCompensation(double currPower);

    }   //interface PowerCompensation

    /**
     * This class encapsulates all the PID coefficients into a single object and makes it more efficient to pass them
     * around.
     */
    public static class PidCoefficients
    {
        public double kP;
        public double kI;
        public double kD;
        public double kF;
        public double iZone;

        /**
         * Constructor: Create an instance of the object.
         *
         * @param kP specifies the Proportional constant.
         * @param kI specifies the Integral constant.
         * @param kD specifies the Differential constant.
         * @param kF specifies the Feed forward constant.
         * @param iZone specifies the integral zone.
         */
        public PidCoefficients(double kP, double kI, double kD, double kF, double iZone)
        {
            this.kP = Math.abs(kP);
            this.kI = Math.abs(kI);
            this.kD = Math.abs(kD);
            this.kF = Math.abs(kF);
            this.iZone = Math.abs(iZone);
        }   //PidCoefficients

        /**
         * Constructor: Create an instance of the object.
         *
         * @param kP specifies the Proportional constant.
         * @param kI specifies the Integral constant.
         * @param kD specifies the Differential constant.
         * @param kF specifies the Feed forward constant.
         */
        public PidCoefficients(double kP, double kI, double kD, double kF)
        {
            this(kP, kI, kD, kF, 0.0);
        }   //PidCoefficients

        /**
         * Constructor: Create an instance of the object.
         *
         * @param kP specifies the Proportional constant.
         * @param kI specifies the Integral constant.
         * @param kD specifies the Differential constant.
         */
        public PidCoefficients(double kP, double kI, double kD)
        {
            this(kP, kI, kD, 0.0, 0.0);
        }   //PidCoefficients

        /**
         * Constructor: Create an instance of the object.
         *
         * @param kP specifies the Proportional constant.
         */
        public PidCoefficients(double kP)
        {
            this(kP, 0.0, 0.0, 0.0, 0.0);
        }   //PidCoefficients

        /**
         * Constructor: Create an instance of the object.
         */
        public PidCoefficients()
        {
            this(1.0, 0.0, 0.0, 0.0, 0.0);
        }   //PidCoefficients

        /**
         * This method returns all PID coefficients in string form.
         *
         * @return PID coefficients string.
         */
        @Override
        public String toString()
        {
            return String.format(Locale.US, "(PIDFiZone:%f,%f,%f,%f,%f)", kP, kI, kD, kF, iZone);
        }   //toString

        /**
         * This method returns a copy of this object.
         *
         * @return a copy of this object.
         */
        public PidCoefficients clone()
        {
            return new PidCoefficients(kP, kI, kD, kF, iZone);
        }   //clone

    }   //class PidCoefficients

    /**
     * This class encapsulates all the PID parameters into a single object and makes it more efficient to pass them
     * around.
     */
    public static class PidParameters
    {
        public PidCoefficients pidCoeff;
        public double tolerance;
        public final double settlingTime;
        public double steadyStateError;
        public double stallErrRateThreshold;
        public PidInput pidInput;
        public PowerCompensation powerCompensation;

        /**
         * Constructor: Create an instance of the object.
         *
         * @param pidCoeff specifies the PID coefficients for the PID controller.
         * @param tolerance specifies the tolerance.
         * @param settlingTime specifies the minimum on target settling time.
         * @param steadyStateError specifies the acceptable steady state error.
         * @param stallErrRateThreshold specifies the error rate below which we would consider PID stalled.
         * @param pidInput specifies the method to call to get PID sensor input.
         * @param powerCompensation specifies the method to call to get power compensation, can be null if not
         *        provided.
         */
        public PidParameters(
            PidCoefficients pidCoeff, double tolerance, double settlingTime, double steadyStateError,
            double stallErrRateThreshold, PidInput pidInput, PowerCompensation powerCompensation)
        {
            this.pidCoeff = pidCoeff;
            this.settlingTime = Math.abs(settlingTime);
            this.stallErrRateThreshold = stallErrRateThreshold;
            setErrorTolerances(tolerance, steadyStateError);
            this.pidInput = pidInput;
            this.powerCompensation = powerCompensation;
        }   //PidParameters

        /**
         * Constructor: Create an instance of the object.
         *
         * @param pidCoeff specifies the PID coefficients for the PID controller.
         * @param tolerance specifies the tolerance.
         * @param settlingTime specifies the minimum on target settling time.
         * @param pidInput specifies the method to call to get PID sensor input.
         * @param powerCompensation specifies the method to call to get power compensation, can be null if not
         *        provided.
         */
        public PidParameters(
            PidCoefficients pidCoeff, double tolerance, double settlingTime, PidInput pidInput,
            PowerCompensation powerCompensation)
        {
            this(pidCoeff, tolerance, settlingTime, tolerance, 0.0, pidInput, powerCompensation);
        }   //PidParameters

        /**
         * Constructor: Create an instance of the object.
         *
         * @param pidCoeff specifies the PID coefficients for the PID controller.
         * @param tolerance specifies the tolerance.
         * @param settlingTime specifies the minimum on target settling time.
         *        provided.
         * @param pidInput specifies the method to call to get PID sensor input.
         */
        public PidParameters(PidCoefficients pidCoeff, double tolerance, double settlingTime, PidInput pidInput)
        {
            this(pidCoeff, tolerance, settlingTime, tolerance, 0.0, pidInput, null);
        }   //PidParameters

        /**
         * Constructor: Create an instance of the object.
         *
         * @param pidCoeff specifies the PID coefficients for the PID controller.
         * @param tolerance specifies the tolerance.
         * @param pidInput specifies the method to call to get PID sensor input.
         * @param powerCompensation specifies the method to call to get power compensation, can be null if not
         *        provided.
         */
        public PidParameters(
            PidCoefficients pidCoeff, double tolerance, PidInput pidInput, PowerCompensation powerCompensation)
        {
            this(pidCoeff, tolerance, DEF_SETTLING_TIME, tolerance, 0.0, pidInput, powerCompensation);
        }   //PidParameters

        /**
         * Constructor: Create an instance of the object.
         *
         * @param pidCoeff specifies the PID coefficients for the PID controller.
         * @param tolerance specifies the tolerance.
         * @param pidInput specifies the method to call to get PID sensor input.
         */
        public PidParameters(PidCoefficients pidCoeff, double tolerance, PidInput pidInput)
        {
            this(pidCoeff, tolerance, DEF_SETTLING_TIME, tolerance, 0.0, pidInput, null);
        }   //PidParameters

        /**
         * Constructor: Create an instance of the object.
         *
         * @param kP specifies the Proportional constant.
         * @param kI specifies the Integral constant.
         * @param kD specifies the Differential constant.
         * @param kF specifies the Feed forward constant.
         * @param iZone specifies the integral zone.
         * @param tolerance specifies the tolerance.
         * @param settlingTime specifies the minimum on target settling time.
         * @param steadyStateError specifies the acceptable steady state error.
         * @param stallErrRateThreshold specifies the error rate below which we would consider PID stalled.
         * @param pidInput specifies the method to call to get PID sensor input.
         * @param powerCompensation specifies the method to call to get power compensation, can be null if not
         *        provided.
         */
        public PidParameters(
            double kP, double kI, double kD, double kF, double iZone, double tolerance, double settlingTime,
            double steadyStateError, double stallErrRateThreshold, PidInput pidInput,
            PowerCompensation powerCompensation)
        {
            this(new PidCoefficients(kP, kI, kD, kF, iZone),
                 tolerance, settlingTime, steadyStateError, stallErrRateThreshold, pidInput, powerCompensation);
        }   //PidParameters

        /**
         * Constructor: Create an instance of the object.
         *
         * @param kP specifies the Proportional constant.
         * @param kI specifies the Integral constant.
         * @param kD specifies the Differential constant.
         * @param kF specifies the Feed forward constant.
         * @param iZone specifies the integral zone.
         * @param tolerance specifies the tolerance.
         * @param settlingTime specifies the minimum on target settling time.
         * @param pidInput specifies the method to call to get PID sensor input.
         */
        public PidParameters(
            double kP, double kI, double kD, double kF, double iZone, double tolerance, double settlingTime,
            PidInput pidInput)
        {
            this(kP, kI, kD, kF, iZone, tolerance, settlingTime, tolerance, 0.0, pidInput, null);
        }   //PidParameters

        /**
         * Constructor: Create an instance of the object.
         *
         * @param kP specifies the Proportional constant.
         * @param kI specifies the Integral constant.
         * @param kD specifies the Differential constant.
         * @param kF specifies the Feed forward constant.
         * @param iZone specifies the integral zone.
         * @param tolerance specifies the tolerance.
         * @param pidInput specifies the method to call to get PID sensor input.
         */
        public PidParameters(
            double kP, double kI, double kD, double kF, double iZone, double tolerance, PidInput pidInput)
        {
            this(kP, kI, kD, kF, iZone, tolerance, DEF_SETTLING_TIME, tolerance, 0.0, pidInput, null);
        }   //PidParameters

        /**
         * Constructor: Create an instance of the object.
         *
         * @param kP specifies the Proportional constant.
         * @param kI specifies the Integral constant.
         * @param kD specifies the Differential constant.
         * @param kF specifies the Feed forward constant.
         * @param tolerance specifies the tolerance.
         * @param pidInput specifies the method to call to get PID sensor input.
         */
        public PidParameters(double kP, double kI, double kD, double kF, double tolerance, PidInput pidInput)
        {
            this(kP, kI, kD, kF, 0.0, tolerance, DEF_SETTLING_TIME, tolerance, 0.0, pidInput, null);
        }   //PidParameters

        /**
         * Constructor: Create an instance of the object.
         *
         * @param kP specifies the Proportional constant.
         * @param kI specifies the Integral constant.
         * @param kD specifies the Differential constant.
         * @param tolerance specifies the tolerance.
         * @param pidInput specifies the method to call to get PID sensor input.
         */
        public PidParameters(double kP, double kI, double kD, double tolerance, PidInput pidInput)
        {
            this(kP, kI, kD, 0.0, 0.0, tolerance, DEF_SETTLING_TIME, tolerance, 0.0, pidInput, null);
        }   //PidParameters

        /**
         * This method sets the method to call to get PID sensor input.
         *
         * @param pidInput specifies the method to call to get PID sensor input.
         */
        public void setPidInput(PidInput pidInput)
        {
            this.pidInput = pidInput;
        }   //setPidInput

        /**
         * This method sets the method to call to get power compensation.
         *
         * @param powerCompensation specifies the method to call to get power compensation.
         */
        public void setPowerCompensation(PowerCompensation powerCompensation)
        {
            this.powerCompensation = powerCompensation;
        }   //setPowerCompensation

        /**
         * This method sets the target tolerance as well as acceptable steady state error. If the PID error is between
         * tolerance and steady state error and the error rate is zero, PID control will consider this is a stall
         * condition (i.e. it won't make it to within tolerance but within acceptable steady state error). By default,
         * steadyStateError is set to be the same as tolerance so that stall detection is effectively disabled. By
         * setting steadyStateError larger than tolerance, the error range between tolerance and steadyStateError will
         * become the stall detection zone in which if the error rate is zero, it will declare PID is stalled. If the
         * PID controller is in stalled state, it is considered OnTarget even though it is not within tolerance. By
         * adjusting steadyStateError, one can prevent the PID controller from hanging indefinitely and not reaching
         * target by declaring OnTarget.
         *
         * @param tolerance specifies the tolerance.
         * @param steadyStateError specifies the acceptable steady state error.
         */
        public void setErrorTolerances(double tolerance, double steadyStateError)
        {
            tolerance = Math.abs(tolerance);
            steadyStateError = Math.abs(steadyStateError);

            if (tolerance > steadyStateError)
            {
                throw new IllegalArgumentException("steadyStateError must not be smaller than tolerance.");
            }

            if (pidCoeff.iZone > 0.0 && tolerance >= pidCoeff.iZone)
            {
                throw new IllegalArgumentException("iZone must be greater than tolerance.");
            }

            this.tolerance = tolerance;
            this.steadyStateError = steadyStateError;
        }   //setErrorTolerances

        /**
         * This method sets the error rate below which we will consider a PID stall.
         *
         * @param stallErrRateThreshold specifies the error rate below which we will consider a PID stall.
         */
        public void setStallErrRateThreshold(double stallErrRateThreshold)
        {
            this.stallErrRateThreshold = Math.abs(stallErrRateThreshold);
        }   //setStallErrRateThreshold

        /**
         * This method returns all PID parameters in string form.
         *
         * @return PID parameters string.
         */
        @Override
        public String toString()
        {
            return String.format(
                Locale.US, "pidCoeff=%s,tolerance=%.3f,settlingTime=%.3f", pidCoeff, tolerance, settlingTime);
        }   //toString

    }   //class PidParameters

    /**
     * PID controller needs input from a feedback device for calculating the output power. Whoever is providing this
     * input must implement this interface.
     */
    public interface PidInput
    {
        /**
         * This method is called by the PID controller to get input data from the feedback device. The feedback
         * device can be motor encoders, gyro, ultrasonic sensor, light sensor etc.
         *
         * @return input value of the feedback device.
         */
        double get();

    }   //interface PidInput

    /**
     * This class stores the PID controller state.
     */
    private static class PidCtrlState
    {
        double settlingStartTime = 0.0;
        double setPoint = 0.0;
        double setPointSign = 1.0;
        double currTime = 0.0;
        double deltaTime = 0.0;
        double input = 0.0;
        double currError = 0.0;
        double errorRate = 0.0;
        double totalError = 0.0;
        double pTerm = 0.0;
        double iTerm = 0.0;
        double dTerm = 0.0;
        double fTerm = 0.0;
        double powerComp = 0.0;
        double output = 0.0;

        /**
         * This method resets the PID controller state.
         */
        void reset()
        {
            settlingStartTime = 0.0;
            setPoint = 0.0;
            setPointSign = 1.0;
            currTime = 0.0;
            deltaTime = 0.0;
            input = 0.0;
            currError = 0.0;
            errorRate = 0.0;
            totalError = 0.0;
            pTerm = 0.0;
            iTerm = 0.0;
            dTerm = 0.0;
            fTerm = 0.0;
            powerComp = 0.0;
            output = 0.0;
        }   //reset

    }   //class PidCtrlState

    private final TrcDashboard dashboard = TrcDashboard.getInstance();
    private final String instanceName;
    private final PidParameters pidParams;

    private boolean inverted = false;
    private boolean absSetPoint = false;
    private boolean noOscillation = false;
    private double minTarget = 0.0;
    private double maxTarget = 0.0;
    private double minOutput = -1.0;
    private double maxOutput = 1.0;
    private double outputLimit = 1.0;
    private Double rampRate = null;
    private final Stack<Double> outputLimitStack = new Stack<>();
    private final PidCtrlState pidCtrlState = new PidCtrlState();

    private TrcDbgTrace debugTracer = null;
    private boolean verboseTrace = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param pidParams specifies the PID parameters.
     */
    public TrcPidController(String instanceName, PidParameters pidParams)
    {
        this.instanceName = instanceName;
        this.pidParams = pidParams;
    }   //TrcPidController

    /**
     * Constructor: Create an instance of the object. This constructor is not public. It is only for classes
     * extending this class (e.g. Cascade PID Controller) that cannot make itself as an input provider in its
     * constructor (Java won't allow it). Instead, we provide another protected method setPidInput so it can
     * set the PidInput outside of the super() call.
     *
     * @param instanceName specifies the instance name.
     * @param pidCoeff specifies the PID constants.
     * @param tolerance specifies the target tolerance.
     * @param settlingTime specifies the minimum on target settling time.
     * @param steadyStateError specifies the acceptable steady state error.
     * @param stallErrRateThreshold specifies the error rate below which we would consider PID stalled.
     * @param pidInput specifies the input provider.
     * @param powerCompensation specifies the method to call to get power compensation, can be null if not provided.
     */
    public TrcPidController(
        String instanceName, PidCoefficients pidCoeff, double tolerance, double settlingTime, double steadyStateError,
        double stallErrRateThreshold, PidInput pidInput, PowerCompensation powerCompensation)
    {
        this(instanceName,
             new PidParameters(
             pidCoeff, tolerance, settlingTime, steadyStateError, stallErrRateThreshold, pidInput, powerCompensation));
    }   //TrcPidController

    /**
     * Constructor: Create an instance of the object. This constructor is not public. It is only for classes
     * extending this class (e.g. Cascade PID Controller) that cannot make itself as an input provider in its
     * constructor (Java won't allow it). Instead, we provide another protected method setPidInput so it can
     * set the PidInput outside of the super() call.
     *
     * @param instanceName specifies the instance name.
     * @param pidCoeff specifies the PID constants.
     * @param tolerance specifies the target tolerance.
     * @param settlingTime specifies the minimum on target settling time.
     * @param steadyStateError specifies the acceptable steady state error.
     * @param stallErrRateThreshold specifies the error rate below which we would consider PID stalled.
     * @param pidInput specifies the input provider.
     */
    public TrcPidController(
        String instanceName, PidCoefficients pidCoeff, double tolerance, double settlingTime, double steadyStateError,
        double stallErrRateThreshold, PidInput pidInput)
    {
        this(instanceName,
             new PidParameters(pidCoeff, tolerance, settlingTime, steadyStateError, stallErrRateThreshold, pidInput,
            null));
    }   //TrcPidController

    /**
     * Constructor: Create an instance of the object. This constructor is not public. It is only for classes
     * extending this class (e.g. Cascade PID Controller) that cannot make itself as an input provider in its
     * constructor (Java won't allow it). Instead, we provide another protected method setPidInput so it can
     * set the PidInput outside of the super() call.
     *
     * @param instanceName specifies the instance name.
     * @param pidCoeff specifies the PID constants.
     * @param tolerance specifies the target tolerance.
     * @param settlingTime specifies the minimum on target settling time.
     * @param pidInput specifies the input provider.
     * @param powerCompensation specifies the method to call to get power compensation, can be null if not provided.
     */
    public TrcPidController(
        String instanceName, PidCoefficients pidCoeff, double tolerance, double settlingTime,
        PidInput pidInput, PowerCompensation powerCompensation)
    {
        this(instanceName, new PidParameters(pidCoeff, tolerance, settlingTime, pidInput, powerCompensation));
    }   //TrcPidController

    /**
     * Constructor: Create an instance of the object. This constructor is not public. It is only for classes
     * extending this class (e.g. Cascade PID Controller) that cannot make itself as an input provider in its
     * constructor (Java won't allow it). Instead, we provide another protected method setPidInput so it can
     * set the PidInput outside of the super() call.
     *
     * @param instanceName specifies the instance name.
     * @param pidCoeff specifies the PID constants.
     * @param tolerance specifies the target tolerance.
     * @param settlingTime specifies the minimum on target settling time.
     * @param pidInput specifies the input provider.
     */
    public TrcPidController(
        String instanceName, PidCoefficients pidCoeff, double tolerance, double settlingTime, PidInput pidInput)
    {
        this(instanceName, new PidParameters(pidCoeff, tolerance, settlingTime, pidInput, null));
    }   //TrcPidController

    /**
     * Constructor: Create an instance of the object. This constructor is not public. It is only for classes
     * extending this class (e.g. Cascade PID Controller) that cannot make itself as an input provider in its
     * constructor (Java won't allow it). Instead, we provide another protected method setPidInput so it can
     * set the PidInput outside of the super() call.
     *
     * @param instanceName specifies the instance name.
     * @param pidCoeff specifies the PID constants.
     * @param tolerance specifies the target tolerance.
     * @param pidInput specifies the input provider.
     * @param powerCompensation specifies the method to call to get power compensation, can be null if not provided.
     */
    public TrcPidController(
        String instanceName, PidCoefficients pidCoeff, double tolerance, PidInput pidInput,
        PowerCompensation powerCompensation)
    {
        this(instanceName, new PidParameters(pidCoeff, tolerance, DEF_SETTLING_TIME, pidInput, powerCompensation));
    }   //TrcPidController

    /**
     * Constructor: Create an instance of the object. This constructor is not public. It is only for classes
     * extending this class (e.g. Cascade PID Controller) that cannot make itself as an input provider in its
     * constructor (Java won't allow it). Instead, we provide another protected method setPidInput so it can
     * set the PidInput outside of the super() call.
     *
     * @param instanceName specifies the instance name.
     * @param pidCoeff specifies the PID constants.
     * @param tolerance specifies the target tolerance.
     * @param pidInput specifies the input provider.
     */
    public TrcPidController(String instanceName, PidCoefficients pidCoeff, double tolerance, PidInput pidInput)
    {
        this(instanceName, new PidParameters(pidCoeff, tolerance, DEF_SETTLING_TIME, pidInput, null));
    }   //TrcPidController

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @Override
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This method returns the PID parameters.
     *
     * @return PID parameters.
     */
    public PidParameters getPidParameters()
    {
        return pidParams;
    }   //getPidParameters

    /**
     * This method returns the PidInput interface.
     *
     * @return PidInput interface.
     */
    public PidInput getPidInput()
    {
        return pidParams.pidInput;
    }   //getPidInput

    /**
     * This method inverts the sign of the calculated error. Normally, the calculated error starts with a large
     * positive number and goes down. However, in some sensors such as the ultrasonic sensor, the target is a small
     * number and the error starts with a negative value and increases. In order to calculate a correct output which
     * will go towards the target, the error sign must be inverted.
     *
     * @param inverted specifies true to invert the sign of the calculated error, false otherwise.
     */
    public void setInverted(boolean inverted)
    {
        synchronized (pidCtrlState)
        {
            this.inverted = inverted;
        }
    }   //setInverted

    /**
     * This method sets the set point mode to be absolute. PID controller always calculates the output with an
     * absolute set point comparing to a sensor value representing an absolute input. But by default, it will
     * treat the set point as a value relative to its current input. So it will add the relative set point value
     * to the current input as the absolute set point in its calculation. This method allows the caller to treat
     * the set point as absolute set point.
     *
     * @param absolute specifies true if set point is absolute, false otherwise.
     */
    public void setAbsoluteSetPoint(boolean absolute)
    {
        synchronized (pidCtrlState)
        {
            this.absSetPoint = absolute;
        }
    }   //setAbsoluteSetPoint

    /**
     * This method returns true if setpoints are absolute, false otherwise.
     *
     * @return true if setpoints are absolute, false otherwise.
     */
    public boolean hasAbsoluteSetPoint()
    {
        return absSetPoint;
    }   //hasAbsoluteSetPoint

    /**
     * This method enables/disables NoOscillation mode. In PID control, if the PID constants are not tuned quite
     * correctly, it may cause oscillation that could waste a lot of time. In some scenarios, passing the target
     * beyond the tolerance may be acceptable. This method allows the PID controller to declare "On Target" even
     * though it passes the target beyond tolerance so it doesn't oscillate.
     *
     * @param noOscillation specifies true to enable no oscillation, false to disable.
     */
    public void setNoOscillation(boolean noOscillation)
    {
        synchronized (pidCtrlState)
        {
            this.noOscillation = noOscillation;
        }
    }   //setNoOscillation

    /**
     * This method returns the current PID coefficients.
     *
     * @return current PID coefficients.
     */
    public PidCoefficients getPidCoefficients()
    {
        return pidParams.pidCoeff;
    }   //getPidCoefficients

    /**
     * This method sets new PID coefficients.
     *
     * @param pidCoeff specifies new PID coefficients.
     */
    public void setPidCoefficients(PidCoefficients pidCoeff)
    {
        synchronized (pidCtrlState)
        {
            this.pidParams.pidCoeff = pidCoeff;
        }
    }   //setPidCoefficients

    /**
     * This method sets the ramp rate of the PID controller output. It is sometimes useful to limit the acceleration
     * of the output of the PID controller. For example, the strafing PID controller on a mecanum drive base may
     * benefit from a lower acceleration to minimize wheel slippage.
     *
     * @param rampRate specifies the ramp rate in percent power per second, null to disable.
     */
    public void setRampRate(Double rampRate)
    {
        synchronized (pidCtrlState)
        {
            this.rampRate = rampRate;
        }
    }   //setRampRate

    /**
     * This method sets the target tolerance as well as acceptable steady state error. If the PID error is between
     * tolerance and steady state error and the error rate is zero, PID control will consider this is a stall
     * condition (i.e. it won't make it to within tolerance but within acceptable steady state error). By default,
     * steadyStateError is set to be the same as tolerance so that stall detection is effectively disabled. By
     * setting steadyStateError larger than tolerance, the error range between tolerance and steadyStateError will
     * become the stall detection zone in which if the error rate is zero, it will declare PID is stalled. If the
     * PID controller is in stalled state, it is considered OnTarget even though it is not within tolerance. By
     * adjusting steadyStateError, one can prevent the PID controller from hanging indefinitely and not reaching
     * target by declaring OnTarget.
     *
     * @param steadyStateError specifies the acceptable steady state error.
     */
    public void setErrorTolerances(double tolerance, double steadyStateError)
    {
        synchronized (pidCtrlState)
        {
            pidParams.setErrorTolerances(tolerance, steadyStateError);
        }
    }   //setErrorTolerances

    /**
     * This method sets a new steady state error to the value of tolerance multiplied by the given multiplier.
     *
     * @param multiplier specifies the tolerance multiplier to calculate the new steadyStateError.
     */
    public void setSteadyStateErrorByMultiplier(double multiplier)
    {
        double tolerance = pidParams.tolerance;
        double steadyStateError = tolerance * Math.abs(multiplier);
        setErrorTolerances(tolerance, steadyStateError);
    }   //setSteadyStateErrorByMultiplier

    /**
     * This method is called by the subclass to set the stall detection error rate threshold value.
     *
     * @param stallErrRateThreshold specifies the stall detection error rate threshold value.
     */
    public void setStallErrRateThreshold(double stallErrRateThreshold)
    {
        synchronized (pidCtrlState)
        {
            pidParams.setStallErrRateThreshold(stallErrRateThreshold);
        }
    }   //setStallVelocityThreshold

    /**
     * This method sets a new target tolerance.
     *
     * @param tolerance specifies the new target tolerance.
     */
    public void setTargetTolerance(double tolerance)
    {
        synchronized (pidCtrlState)
        {
            pidParams.tolerance = Math.abs(tolerance);
        }
    }   //setTargetTolerance

    /**
     * This method sets a range limit on the target set point.
     *
     * @param minTarget specifies the target set point lower range limit.
     * @param maxTarget specifies the target set point higher range limit.
     */
    public void setTargetRange(double minTarget, double maxTarget)
    {
        synchronized (pidCtrlState)
        {
            this.minTarget = minTarget;
            this.maxTarget = maxTarget;
        }
    }   //setTargetRange

    /**
     * This method sets a range limit on the calculated output. It is very useful to limit the output range to
     * less than full power for scenarios such as using mecanum wheels on a drive train to prevent wheel slipping
     * or slow down a PID drive in order to detect a line etc.
     *
     * @param minOutput specifies the PID output lower range limit.
     * @param maxOutput specifies the PID output higher range limit.
     */
    public void setOutputRange(double minOutput, double maxOutput)
    {
        if (maxOutput <= minOutput)
        {
            throw new IllegalArgumentException("maxOutput must be greater than minOutput");
        }

        synchronized (pidCtrlState)
        {
            if (Math.abs(minOutput) == Math.abs(maxOutput))
            {
                outputLimit = maxOutput;
            }

            this.minOutput = minOutput;
            this.maxOutput = maxOutput;
        }
    }   //setOutputRange

    /**
     * This method sets the output to the range -limit to +limit. It calls setOutputRange. If the caller wants
     * to limit the output power symmetrically, this is the method to call, not setOutputRange.
     *
     * @param limit specifies the output limit as a positive number.
     */
    public void setOutputLimit(double limit)
    {
        synchronized (pidCtrlState)
        {
            limit = Math.abs(limit);
            setOutputRange(-limit, limit);
        }
    }   //setOutputLimit

    /**
     * This method returns the last set output limit. It is sometimes useful to temporarily change the output
     * range of the PID controller for an operation and restore it afterwards. This method allows the caller to
     * save the last set output limit and restore it later on.
     *
     * @return last set output limit.
     */
    public double getOutputLimit()
    {
        return outputLimit;
    }   //getOutputLimit

    /**
     * This method saves the current output limit of the PID controller and sets it to the given new limit.
     * This is useful if the caller wants to temporarily set a limit for an operation and restore it afterwards.
     * Note: this is implemented with a stack so it is assuming the saving and restoring calls are nested in
     * nature. If this is called in a multi-threading environment where saving and restoring can be interleaved
     * by different threads, unexpected result may happen. It is recommended to avoid this type of scenario if
     * possible.
     *
     * @param limit specifies the new output limit.
     * @return return the previous output limit.
     */
    public double saveAndSetOutputLimit(double limit)
    {
        double prevLimit;

        synchronized (pidCtrlState)
        {
            prevLimit = outputLimit;
            outputLimitStack.push(outputLimit);
            setOutputLimit(limit);
        }

        return prevLimit;
    }   //saveAndSetOutputLimit

    /**
     * This method restores the last saved output limit and return its value. If there was no previous call to
     * saveAndSetOutputLimit, the current output limit is returned and the limit is not changed.
     *
     * @return last saved output limit.
     */
    public double restoreOutputLimit()
    {
        double limit;

        synchronized (pidCtrlState)
        {
            try
            {
                limit = outputLimitStack.pop();
                setOutputLimit(limit);
            }
            catch (EmptyStackException e)
            {
                //
                // There was no previous saveAndSetOutputLimit call, don't do anything and just return the current
                // output limit.
                //
                limit = outputLimit;
            }
        }

        return limit;
    }   //restoreOutputLimit

    /**
     * This method returns the current set point value.
     *
     * @return current set point.
     */
    public double getTarget()
    {
        return pidCtrlState.setPoint;
    }   //getTarget

    /**
     * This methods sets the target set point.
     *
     * @param target    specifies the target set point.
     * @param warpSpace specifies the warp space object if the target is in one, null if not.
     * @param resetError specifies true to reset error state, false otherwise. It is important to preserve error
     *                   state if we are changing target before the PID operation is completed.
     */
    public void setTarget(double target, TrcWarpSpace warpSpace, boolean resetError)
    {
        //
        // Read from input device without holding a lock on this object, since this could
        // be a long-running call.
        // Note: if we are changing target, don't need to waste time to get current input because we are not
        // updating error states anyway.
        //
        final double input = resetError? pidParams.pidInput.get(): 0.0;

        synchronized (pidCtrlState)
        {
            double error;

            if (resetError)
            {
                pidCtrlState.input = input;
            }

            if (!absSetPoint)
            {
                //
                // Set point is relative, add target to current input to get absolute set point.
                //
                pidCtrlState.setPoint = pidCtrlState.input + target;
                error = target;
            }
            else
            {
                //
                // Set point is absolute, use as is but optimize it if it is in warp space.
                //
                pidCtrlState.setPoint = target;
                if (warpSpace != null)
                {
                    pidCtrlState.setPoint = warpSpace.getOptimizedTarget(pidCtrlState.setPoint, pidCtrlState.input);
                }
                error = pidCtrlState.setPoint - pidCtrlState.input;
            }

            if (inverted)
            {
                error *= -1.0;
            }
            //
            // If there is a valid target range, limit the set point to this range.
            //
            if (maxTarget > minTarget)
            {
                if (pidCtrlState.setPoint > maxTarget)
                {
                    pidCtrlState.setPoint = maxTarget;
                }
                else if (pidCtrlState.setPoint < minTarget)
                {
                    pidCtrlState.setPoint = minTarget;
                }
            }

            if (resetError)
            {
                pidCtrlState.currError = error;
                pidCtrlState.errorRate = 0.0;
                pidCtrlState.totalError = 0.0;
                pidCtrlState.setPointSign = Math.signum(error);

                pidCtrlState.currTime = pidCtrlState.settlingStartTime = TrcTimer.getCurrentTime();
            }
        }
    }   //setTarget

    /**
     * This methods sets the target set point.
     *
     * @param target    specifies the target set point.
     * @param warpSpace specifies the warp space object if the target is in one, null if not.
     */
    public void setTarget(double target, TrcWarpSpace warpSpace)
    {
        setTarget(target, warpSpace, true);
    }   //setTarget

    /**
     * This methods sets the target set point.
     *
     * @param target    specifies the target set point.
     * @param resetError specifies true to reset error state, false otherwise. It is important to preserve error
     *                   state if we are changing target before the PID operation is completed.
     */
    public void setTarget(double target, boolean resetError)
    {
        setTarget(target, null, resetError);
    }   //setTarget

    /**
     * This methods sets the target set point.
     *
     * @param target specifies the target set point.
     */
    public void setTarget(double target)
    {
        setTarget(target, null, true);
    }   //setTarget

    /**
     * This method returns the error of a previous output calculation.
     *
     * @return previous error.
     */
    public double getError()
    {
        return pidCtrlState.currError;
    }   //getError

    /**
     * This method resets the PID controller clearing the set point, error, total error and output.
     */
    public void reset()
    {
        synchronized (pidCtrlState)
        {
            pidCtrlState.reset();
        }
    }   //reset

    /**
     * This method determines if we have reached the set point target. It is considered on target if the previous
     * error is smaller than the tolerance and there is no movement for at least settling time. If NoOscillation mode
     * is set, it is considered on target if we are within tolerance or pass target regardless of setting time.
     *
     * @return true if we reached target, false otherwise.
     */
    public boolean isOnTarget()
    {
        final String funcName = "isOnTarget";
        boolean onTarget = false;

        synchronized (pidCtrlState)
        {
            double currTime = TrcTimer.getCurrentTime();
            double absErr = Math.abs(pidCtrlState.currError);

            if (noOscillation)
            {
                //
                // Don't allow oscillation, so if we are within tolerance or we pass target, just quit.
                // If setPointSign is positive, it means the target is "forward". So if currError <= tolerance,
                // it means we are either within tolerance or have passed the target.
                // If setPointSign is negative, it means the target is "backward". So if -currError <= tolerance,
                // it means we are either within tolerance or have passed the target.
                //
                if (pidCtrlState.currError*pidCtrlState.setPointSign <= pidParams.tolerance)
                {
                    onTarget = true;
                }
            }
            //
            // We consider it on-target if error is within tolerance for the settling period.
            //
            else if (absErr > pidParams.steadyStateError ||
                     absErr > pidParams.tolerance &&
                     Math.abs(pidCtrlState.errorRate) > pidParams.stallErrRateThreshold)
            {
                pidCtrlState.settlingStartTime = TrcTimer.getCurrentTime();

                if (debugEnabled)
                {
                    globalTracer.traceInfo(
                        funcName,
                        "err=%.3f, errRate=%.3f, tolerance=%.1f, steadyStateErr=%.1f, stallErrRateThreshold=%.1f",
                        pidCtrlState.currError, pidCtrlState.errorRate, pidParams.tolerance,
                        pidParams.steadyStateError, pidParams.stallErrRateThreshold);
                }
            }
            else if (currTime >= pidCtrlState.settlingStartTime + pidParams.settlingTime)
            {
                if (debugEnabled)
                {
                    globalTracer.traceInfo(
                        funcName, "currTime=%.3f, startTime=%.3f, err=%.3f, errRate=%.3f, tolerance=%.1f, " +
                        "steadyStateErr=%.1f, stallErrRateThreshold=%.1f",
                        currTime, pidCtrlState.settlingStartTime, pidCtrlState.currError, pidCtrlState.errorRate,
                        pidParams.tolerance, pidParams.steadyStateError, pidParams.stallErrRateThreshold);
                }

                onTarget = true;
            }
        }

        return onTarget;
    }   //isOnTarget

    /**
     * This method returns the current PID input value.
     *
     * @return current PID input value.
     */
    public double getCurrentInput()
    {
        return pidParams.pidInput.get();
    }   //getCurrentInput

    /**
     * This method calculates the PID output applying the PID equation to the given set point target and current
     * input value.
     *
     * @return PID output value.
     */
    public double getOutput()
    {
        //
        // Read from input device without holding a lock on this object, since this could
        // be a long-running call.
        //
        final double currInput = pidParams.pidInput.get();

        synchronized (pidCtrlState)
        {
            double prevTime = pidCtrlState.currTime;
            pidCtrlState.currTime = TrcTimer.getCurrentTime();
            pidCtrlState.deltaTime = pidCtrlState.currTime - prevTime;

            double prevError = pidCtrlState.currError;
            pidCtrlState.currError = inverted? currInput - pidCtrlState.setPoint: pidCtrlState.setPoint - currInput;
            pidCtrlState.errorRate =
                pidCtrlState.deltaTime > 0.0? (pidCtrlState.currError - prevError)/pidCtrlState.deltaTime: 0.0;
            double absErr = Math.abs(pidCtrlState.currError);

            pidCtrlState.input = currInput;
            // Only allow integration if error is within iZone but greater than tolerance.
            if (pidParams.pidCoeff.kI != 0.0 &&
                (pidParams.pidCoeff.iZone == 0.0 ||
                 absErr <= pidParams.pidCoeff.iZone && absErr > pidParams.tolerance))
            {
                //
                // Make sure the total error doesn't get wound up too much exceeding maxOutput.
                // This is essentially capping the I-term to within the range of minOutput and maxOutput.
                //
                double potentialGain =
                    (pidCtrlState.totalError + pidCtrlState.currError * pidCtrlState.deltaTime) * pidParams.pidCoeff.kI;
                if (potentialGain >= maxOutput)
                {
                    pidCtrlState.totalError = maxOutput / pidParams.pidCoeff.kI;
                }
                else if (potentialGain > minOutput)
                {
                    pidCtrlState.totalError += pidCtrlState.currError * pidCtrlState.deltaTime;
                }
                else
                {
                    pidCtrlState.totalError = minOutput / pidParams.pidCoeff.kI;
                }
            }
            else
            {
                pidCtrlState.totalError = 0.0;
            }

            pidCtrlState.pTerm = pidParams.pidCoeff.kP * pidCtrlState.currError;
            pidCtrlState.iTerm = pidParams.pidCoeff.kI * pidCtrlState.totalError;
            pidCtrlState.dTerm = pidParams.pidCoeff.kD * pidCtrlState.errorRate;
            pidCtrlState.fTerm = pidParams.pidCoeff.kF * pidCtrlState.setPoint;

            double output = TrcUtil.clipRange(
                pidCtrlState.pTerm + pidCtrlState.iTerm + pidCtrlState.dTerm + pidCtrlState.fTerm,
                minOutput, maxOutput);

            if (rampRate != null)
            {
                double maxChange = rampRate * pidCtrlState.deltaTime;
                double change = TrcUtil.clipRange(output - pidCtrlState.output, -maxChange, maxChange);
                output = pidCtrlState.output + change;
            }

            if (pidParams.powerCompensation != null)
            {
                pidCtrlState.powerComp = pidParams.powerCompensation.getCompensation(output);
                output = TrcUtil.clipRange(output + pidCtrlState.powerComp, minOutput, maxOutput);
            }
            pidCtrlState.output = output;

            if (debugTracer != null)
            {
                printPidInfo(debugTracer, verboseTrace);
            }

            return pidCtrlState.output;
        }
    }   //getOutput

    /**
     * This method displays the PID information on the dashboard for debugging and tuning purpose. Note that the
     * PID info occupies two dashboard lines.
     *
     * @param lineNum specifies the starting line number of the dashboard to display the info.
     */
    public void displayPidInfo(int lineNum)
    {
        synchronized (pidCtrlState)
        {
            dashboard.displayPrintf(
                lineNum, "%s:Target=%.1f,Input=%.1f,Error=%.1f",
                instanceName, pidCtrlState.setPoint, pidCtrlState.input, pidCtrlState.currError);
            dashboard.displayPrintf(
                lineNum + 1, "minOutput=%.1f,Output=%.1f,maxOutput=%.1f",
                minOutput, pidCtrlState.output, maxOutput);
        }
    }   //displayPidInfo

    /**
     * This method prints the PID information to the tracer console. If no tracer is provided, it will attempt to
     * use the debug tracer in this module but if the debug tracer is not enabled, no output will be produced.
     *
     * @param tracer specifies the tracer object to print the PID info to.
     * @param verbose specifies true to print verbose info, false to print summary info.
     * @param battery specifies the battery object to get battery info, can be null if not provided.
     */
    public void printPidInfo(TrcDbgTrace tracer, boolean verbose, TrcRobotBattery battery)
    {
        final String funcName = "printPidInfo";

        if (tracer == null)
        {
            tracer = globalTracer;
        }
        //
        // Apparently, String.format is very expensive. It costs about 5 msec per call for an Android device. In the
        // worst case, the commented code below makes 3 calls to String.format that costs about 15 msec!
        //
        if (tracer != null)
        {
            synchronized (pidCtrlState)
            {
                if (verbose)
                {
                    if (battery != null)
                    {
                        tracer.traceInfo(
                            funcName,
                            "[%.6f] %s: Target=%6.1f, Input=%6.1f, dT=%.6f, CurrErr=%6.1f, ErrRate=%6.1f" +
                            ", Output=%6.3f(%6.3f/%6.3f), PIDFTerms=%6.3f/%6.3f/%6.3f/%6.3f, PowerComp=%6.3f" +
                            ", Volt=%.1f(%.1f)",
                            TrcTimer.getModeElapsedTime(), instanceName, pidCtrlState.setPoint, pidCtrlState.input,
                            pidCtrlState.deltaTime, pidCtrlState.currError, pidCtrlState.errorRate,
                            pidCtrlState.output, minOutput, maxOutput,
                            pidCtrlState.pTerm, pidCtrlState.iTerm, pidCtrlState.dTerm, pidCtrlState.fTerm,
                            pidCtrlState.powerComp, battery.getVoltage(), battery.getLowestVoltage());
                    }
                    else
                    {
                        tracer.traceInfo(
                            funcName,
                            "[%.6f] %s: Target=%6.1f, Input=%6.1f, dT=%.6f, CurrErr=%6.1f, ErrRate=%6.1f" +
                            ", Output=%6.3f(%6.3f/%6.3f), PIDFTerms=%6.3f/%6.3f/%6.3f/%6.3f, powerComp=%6.3f",
                            TrcTimer.getModeElapsedTime(), instanceName, pidCtrlState.setPoint, pidCtrlState.input,
                            pidCtrlState.deltaTime, pidCtrlState.currError, pidCtrlState.errorRate,
                            pidCtrlState.output, minOutput, maxOutput,
                            pidCtrlState.pTerm, pidCtrlState.iTerm, pidCtrlState.dTerm, pidCtrlState.fTerm,
                            pidCtrlState.powerComp);
                    }
                }
                else
                {
                    if (battery != null)
                    {
                        tracer.traceInfo(
                            funcName,
                            "[%.6f] %s: Target=%6.1f, Input=%6.1f, dT=%.6f, CurrErr=%6.1f, ErrRate=%6.1f" +
                            ", Output=%6.3f(%6.3f/%6.3f)" +
                            ", Volt=%.1f(%.1f)",
                            TrcTimer.getModeElapsedTime(), instanceName, pidCtrlState.setPoint, pidCtrlState.input,
                            pidCtrlState.deltaTime, pidCtrlState.currError, pidCtrlState.errorRate,
                            pidCtrlState.output, minOutput, maxOutput,
                            battery.getVoltage(), battery.getLowestVoltage());
                    }
                    else
                    {
                        tracer.traceInfo(
                            funcName,
                            "[%.6f] %s: Target=%6.1f, Input=%6.1f, dT=%.6f, CurrErr=%6.1f, ErrRate=%6.1f" +
                            ", Output=%6.3f(%6.3f/%6.3f)",
                            TrcTimer.getModeElapsedTime(), instanceName, pidCtrlState.setPoint, pidCtrlState.input,
                            pidCtrlState.deltaTime, pidCtrlState.currError, pidCtrlState.errorRate,
                            pidCtrlState.output, minOutput, maxOutput);
                    }
                }
//                StringBuilder msg = new StringBuilder();
//
//                msg.append(String.format(
//                    Locale.US, "[%.6f] %s: Target=%6.1f, Input=%6.1f, Error=%6.1f, Output=%6.3f(%6.3f/%5.3f)",
//                    TrcUtil.getModeElapsedTime(), instanceName, pidCtrlState.setPoint, pidCtrlState.input,
//                    pidCtrlState.error, pidCtrlState.output, minOutput, maxOutput));
//
//                if (verbose)
//                {
//                    msg.append(String.format(
//                        Locale.US, ", PIDFTerms=%6.3f/%6.3f/%6.3f/%6.3f [%.6f/%.6f]",
//                        pidCtrlState.pTerm, pidCtrlState.iTerm, pidCtrlState.dTerm, pidCtrlState.fTerm,
//                        pidCtrlState.deltaTime, TrcUtil.getModeElapsedTime(pidCtrlState.timestamp)));
//                }
//
//                if (battery != null)
//                {
//                    msg.append(String.format(Locale.US, ", Volt=%.1f(%.1f)",
//                        battery.getVoltage(), battery.getLowestVoltage()));
//                }
//
//                tracer.traceInfo(funcName, msg.toString());
            }
        }
    }   //printPidInfo

    /**
     * This method prints the PID information to the tracer console. If no tracer is provided, it will attempt to
     * use the debug tracer in this module but if the debug tracer is not enabled, no output will be produced.
     *
     * @param tracer specifies the tracer object to print the PID info to.
     * @param verbose specifies true to print verbose info, false to print summary info.
     */
    public void printPidInfo(TrcDbgTrace tracer, boolean verbose)
    {
        printPidInfo(tracer, verbose, null);
    }   //printPidInfo

    /**
     * This method prints the PID information to the tracer console. If no tracer is provided, it will attempt to
     * use the debug tracer in this module but if the debug tracer is not enabled, no output will be produced.
     *
     * @param tracer specifies the tracer object to print the PID info to.
     */
    public void printPidInfo(TrcDbgTrace tracer)
    {
        printPidInfo(tracer, false, null);
    }   //printPidInfo

    /**
     * This method prints the PID information to the default debug tracer.
     */
    public void printPidInfo()
    {
        printPidInfo(null, false, null);
    }   //printPidInfo

    /**
     * This method allows the caller to dynamically enable/disable debug tracing of the output calculation. It is
     * very useful for debugging or tuning PID control.
     *
     * @param tracer  specifies the tracer to be used for debug tracing.
     * @param enabled specifies true to enable the debug tracer, false to disable.
     * @param verbose specifies true to enable verbose trace mode, false to disable.
     */
    public void setDebugTraceEnabled(TrcDbgTrace tracer, boolean enabled, boolean verbose)
    {
        synchronized (pidCtrlState)
        {
            debugTracer = enabled ? tracer : null;
            verboseTrace = enabled && verbose;
        }
    }   //setDebugTraceEnabled

    /**
     * This method allows the caller to dynamically enable/disable debug tracing of the output calculation. It is
     * very useful for debugging or tuning PID control.
     *
     * @param tracer  specifies the tracer to be used for debug tracing.
     * @param enabled specifies true to enable the debug tracer, false to disable.
     */
    public void setDebugTraceEnabled(TrcDbgTrace tracer, boolean enabled)
    {
        setDebugTraceEnabled(tracer, enabled, false);
    }   //setDebugTraceEnabled

}   //class TrcPidController
