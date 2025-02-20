#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/adi.hpp"
#include "pros/rtos.hpp"

// Global variables
std::string auton = "SIMPLE";  // Default autonomous routine
int autonIndex = 0;  // Current auton selection index

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Motor groups
pros::MotorGroup leftMotors({-4, -5, -6}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({7, 8, 9}, pros::MotorGearset::blue);

// Individual motors
pros::Motor intake(-10);

// Pneumatics (for Mobile Goal Lift)
pros::adi::Pneumatics mogo('A', false);

// Inertial sensor for orientation tracking
pros::Imu imu(3);

// Tracking wheel encoders (used to track position)
pros::Rotation horizontalEnc(0);  
pros::Rotation verticalEnc(0);    

// Tracking wheels for odometry
constexpr double HORIZONTAL_OFFSET = 5.75;  
constexpr double VERTICAL_OFFSET = 2.5;     
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_325, HORIZONTAL_OFFSET);
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_325, VERTICAL_OFFSET);

// Drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 15, lemlib::Omniwheel::NEW_325, 450, 2);

// PID Controllers for movement correction
lemlib::ControllerSettings linearController(20, 0.001, 7, 0, 0.5, 250, 2.0, 1500, 0);
lemlib::ControllerSettings angularController(2, 0.005, 9, 0, 0.5, 300, 2, 1000, 0);

// Odometry Sensors
lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, &imu);

// Input response curves for smoother driving
lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.019);
lemlib::ExpoDriveCurve steerCurve(3, 10, 1.019);

// Chassis object
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

// Auton names for easy selection
const std::vector<std::string> autonNames = {"SIMPLE", "+RED", "-RED", "+BLUE", "-BLUE", "SKILLS"};

/**
 * Initializes robot systems and starts background tasks.
 */
void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();

    // Background task to update brain screen with telemetry
    pros::Task screenTask([] {
        chassis.setPose(0, 0, 0);
        while (true) {
            pros::lcd::print(0, "X: %.2f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %.2f", -chassis.getPose().y);
            pros::lcd::print(2, "Heading: %.2f", chassis.getPose().theta);
            pros::lcd::print(3, "Auton Mode: %s", auton.c_str());
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled.
 */
void disabled() {}

/**
 * Runs after initialize() if connected to field control.
 * Used for auton selection.
 */
void competition_initialize() {
    if (controller.get_digital_new_press(DIGITAL_A)) {
        autonIndex = (autonIndex + 1) % autonNames.size();
        auton = autonNames[autonIndex];
    }
}
ASSET(test_txt);//change . to _ to make c++ happy :)

/**
 * Runs during autonomous mode.
 */
void autonomous() {
    chassis.setPose(0, 0, 0);

    if (auton == "SIMPLE") {
        chassis.moveToPoint(0, -40, 5000, {.forwards = false});  // Move backward 40 inches
        mogo.extend();  // Activate mogo clamp
        intake.move_voltage(12000);
        pros::delay(500);
        intake.move(0);
    } 
    else if (auton == "+RED") {
        chassis.moveToPoint(0, -12, 5000, {.forwards = false});
        chassis.follow(test_txt, 10, 4000, false);
    }
}

/**
 * Toggles intake on/off.
 */
void toggleIntake() {
    static bool running = false;
    running = !running;
    intake.move_voltage(running ? 12000 : 0);
}

/**
 * Toggles intake reverse on/off.
 */
void toggleReverseIntake() {
    static bool reversing = false;
    reversing = !reversing;
    intake.move_voltage(reversing ? -12000 : 0);
}

/**
 * Toggles mogo clamp.
 */
void toggleMogo() {
    static bool mogoExtended = false;
    mogoExtended = !mogoExtended;
    mogoExtended ? mogo.extend() : mogo.retract();
}

/**
 * Runs in driver control mode.
 */
void opcontrol() {
    auton = "DRIVER";

    while (true) {
        // Arcade drive control
        chassis.arcade(
            controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X),
            controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)
        );

        // Controls
        if (controller.get_digital_new_press(DIGITAL_R1)) toggleIntake();
        if (controller.get_digital_new_press(DIGITAL_R2)) toggleReverseIntake();
        if (controller.get_digital_new_press(DIGITAL_L1)) toggleMogo();

        // Auton selection
        if (controller.get_digital_new_press(DIGITAL_A)) {
            autonIndex = (autonIndex + 1) % autonNames.size();
            auton = autonNames[autonIndex];
        }

        pros::delay(10);
    }
}
