package frc.robot.utils.control;

import java.util.function.Supplier;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.LinearSystem;


/**
 * Combines a state-space model and a Linear-Quadratic Regulator to provide feedforward inputs to a holonomic drivetrain
 * 
 * State-space model description
 * Inputs: x velocity, y velocity, angular velocity (field-oriented)
 * States: x position, y position, rotation
 * A linear state-space model can be represented by the following equation:
 * 
 * xdot = Ax + Bu
 * 
 * y = Cx + Du 
 * 
 * where xdot is the state's rate of change, y is the system's output, x is the system's current state, and u is the input
 * 
 * In field-oriented mode, the controller outputs field-oriented ChassisSpeeds objects,
 * while in chassis-oriented mode, the controller outputs chassis-oriented ChassisSpeeds objects
 */
public class HolonomicDriveLQR {
    private final LinearSystem<N3,N3,N3> drivePlant; // A state-space system representing the robot's drivetrain
    private final LinearQuadraticRegulator<N3,N3,N3> LQRController;
    private boolean fieldOriented;
    private Supplier<Pose2d> poseSupplier; // A supplier providing field-oriented rotation estimates

    /**
     * Constructor for an LQR controlled Holonomic Drive Train. All non-dimensionless values should be given in SI base units (seconds, meters, etc.)
     * 
     * @param Q a vector representing the weights for output error (A higher value will cause the controller to more strongly correct for error)
     * @param R a vector representing the weights for input effort (A higher value will cause the controller to more strongly dampen input)
     * @param PoseSupplier a supplier that provides the robot's current pose
     * @param dt discretization ime step, in seconds
     * @param fieldOriented whether or not the controller should output field-oriented outputs
     */
    public HolonomicDriveLQR(Vector<N3> Q, Vector<N3> R, Supplier<Pose2d> poseSupplier, double dt, boolean fieldOriented) {

        Matrix<N3,N3> A = new Matrix<>(
            new SimpleMatrix(
                new double[][] {
                    {0, 0, 0},
                    {0, 0, 0},
                    {0, 0, 0}
                }
            )
        );

        Matrix<N3,N3> B = new Matrix<>(
            new SimpleMatrix(
                new double[][] {
                    {1, 0, 0},
                    {0, 1, 0},
                    {0, 0, 1}
                }
            )
        );

        Matrix<N3,N3> C = new Matrix<>(
            new SimpleMatrix(
                new double[][] {
                    {1, 0, 0},
                    {0, 1, 0},
                    {0, 0, 1}
                }
            )
        );

        Matrix<N3,N3> D = new Matrix<>(
            new SimpleMatrix(
                new double[][] {
                    {dt, 0, 0},
                    {0, dt, 0},
                    {0, 0, dt}
                }
            )
        );

        drivePlant = new LinearSystem<>(A,B,C,D); // Models field-oriented state with field-oriented speeds as inputs, as the dynamical system is only nonlinear relative to the chassis
        LQRController = new LinearQuadraticRegulator<>(drivePlant, Q, R, dt);
        this.fieldOriented = fieldOriented;
        this.poseSupplier = poseSupplier;
    }

    /**
     * Constructor for an LQR controlled Holonomic Drive Train. All non-dimensionless values should be given in SI base units (seconds, meters, etc.)
     * 
     * @param errorPenalty the error penalty (A higher value will cause the controller to more strongly correct for error)
     * @param inputPenalty the input penalty (A higher value will cause the controller to more strongly dampen input)
     * @param PoseSupplier a supplier that provides the robot's current pose 
     * @param dt discretization time step, in seconds
     * @param fieldOriented whether or not the controller should output field-oriented outputs
     */
    public HolonomicDriveLQR(double errorPenalty, double inputPenalty, Supplier<Pose2d> poseSupplier, double dt, boolean fieldOriented) {
        this(
            VecBuilder.fill(errorPenalty,errorPenalty,errorPenalty),
            VecBuilder.fill(inputPenalty,inputPenalty,inputPenalty),
            poseSupplier,
            dt,
            fieldOriented);
    }

    // Enables and disables field oriented mode
    public void enableFieldOriented(boolean fieldOriented){
        this.fieldOriented = fieldOriented;
    }

    // Returns whether or not the controller is in field-oriented mode
    public boolean isFieldOriented(){
        return fieldOriented;
    }

    /* Returns a state-space model of the plant */
    public LinearSystem<N3,N3,N3> getDrivePlant(){
        return drivePlant;
    }

    /* Returns an LQR controller */
    public LinearQuadraticRegulator<N3,N3,N3> getController(){
        return LQRController;
    }

    /**
     *  Gets the next calculation of the LQR controller
     * 
     * @param _state the robot's current pose
     * @param _setpoint the pose you want the robot to go to
     * @return
     * a field-oriented ChassisSpeeds object if field-oriented is true, and a chassis-oriented ChassisSpeeds object if field-oriented is false
    */
    public ChassisSpeeds calculate(Pose2d _state, Pose2d _setpoint){
        var refVector = StateSpaceUtil.poseToVector(_setpoint);
        var state = StateSpaceUtil.poseToVector(_state);
        var outputFieldRelative = LQRController.calculate(state, refVector);
        if (fieldOriented){
            return new ChassisSpeeds(outputFieldRelative.get(0,0),outputFieldRelative.get(1,0),outputFieldRelative.get(2,0));
        } else {
            return ChassisSpeeds.fromRobotRelativeSpeeds(outputFieldRelative.get(0,0),outputFieldRelative.get(1,0),outputFieldRelative.get(2,0),_state.getRotation());
        }
    }

    /**resets the LQR controller*/
    public void resetController() {
        LQRController.reset();
    }

    /**
     * Adjusts LQR controller to correct for latency
     * 
     * @param dt discretization time-step (seconds)
     * @param ls latency (seconds)
     */
    public void latencyCompensate(double dt, double ls) {
        LQRController.latencyCompensate(drivePlant, dt, ls);
    }

}
