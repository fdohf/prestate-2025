#include "main.h"

// Chassis constructor
ez::Drive chassis(
    {-17, -19, 20},     // Left Chassis Ports 
    {14, 16, -15},  // Right Chassis Ports 

    7,      // IMU Port
    3.25,  // Wheel Diameter 
    360);   // Wheel RPM



void initialize() {
  ez::ez_template_print();
  pros::delay(500);  
  
  chassis.opcontrol_curve_buttons_toggle(true);   
  chassis.opcontrol_drive_activebrake_set(0.0);   
  chassis.opcontrol_curve_default_set(0.0, 0.0);  

  default_constants();


  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
      {"Drive\n\nDrive forward and come back", drive_example},
      {"Turn\n\nTurn 3 times.", turn_example},
      {"Drive and Turn\n\nDrive forward, turn, come back", drive_and_turn},
      {"Drive and Turn\n\nSlow down during drive", wait_until_change_speed},
      {"Swing Turn\n\nSwing in an 'S' curve", swing_example},
      {"Motion Chaining\n\nDrive forward, turn, and come back, but blend everything together :D", motion_chaining},
      {"Combine all 3 movements", combining_movements},
      {"Interference\n\nAfter driving forward, robot performs differently if interfered or not", interfered_example},
      {"Simple Odom\n\nThis is the same as the drive example, but it uses odom instead!", odom_drive_example},
      {"Pure Pursuit\n\nGo to (0, 30) and pass through (6, 10) on the way.  Come back to (0, 0)", odom_pure_pursuit_example},
      {"Pure Pursuit Wait Until\n\nGo to (24, 24) but start running an intake once the robot passes (12, 24)", odom_pure_pursuit_wait_until_example},
      {"Boomerang\n\nGo to (0, 24, 45) then come back to (0, 0, 0)", odom_boomerang_example},
      {"Boomerang Pure Pursuit\n\nGo to (0, 24, 45) on the way to (24, 24) then come back to (0, 0, 0)", odom_boomerang_injected_pure_pursuit_example},
      {"Measure Offsets\n\nThis will turn the robot a bunch of times and calculate your offsets for your tracking wheels.", measure_offsets},
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
}

void disabled() {
  // . . .
}

void competition_initialize() {
  // . . .
}

void autonomous() {
  chassis.pid_targets_reset();                
  chassis.drive_imu_reset();                 
  chassis.drive_sensor_reset();               
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  
  

  ez::as::auton_selector.selected_auton_call();  
}

void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());             
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get()); 
  }
  ez::screen_print(tracker_value + tracker_width, line);  
}

void ez_screen_task() {
  while (true) {
    if (!pros::competition::is_connected()) {
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        if (ez::as::page_blank_is_on(0)) {
          ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
                           1);  

          screen_print_tracker(chassis.odom_tracker_left, "l", 4);
          screen_print_tracker(chassis.odom_tracker_right, "r", 5);
          screen_print_tracker(chassis.odom_tracker_back, "b", 6);
          screen_print_tracker(chassis.odom_tracker_front, "f", 7);
        }
      }
    }

    else {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task ezScreenTask(ez_screen_task);

void ez_template_extras() {
  // Only run this when not connected to a competition switch
  if (!pros::competition::is_connected()) {
    // PID Tuner
    // - after you find values that you're happy with, you'll have to set them in auton.cpp

    // Enable / Disable PID Tuner
    //  When enabled:
    //  * use A and Y to increment / decrement the constants
    //  * use the arrow keys to navigate the constants
    if (master.get_digital_new_press(DIGITAL_X))
      chassis.pid_tuner_toggle();

    // Trigger the selected autonomous routine
    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }

    // Allow PID Tuner to iterate
    chassis.pid_tuner_iterate();
  }

  // Disable PID Tuner when connected to a comp switch
  else {
    if (chassis.pid_tuner_enabled())
      chassis.pid_tuner_disable();
  }
}

bool isRedTeam = true;  // Default to Red Team

// Function to detect and sort rings
void sortRings() {
    colorSensor.set_led_pwm(100);  // Turn on LED for better color detection
    
    while (true) {
        double hue = colorSensor.get_hue();  //  Read hue directly

        // Define hue ranges for red and blue
        bool detectedRed = (hue >= 330 || hue <= 30);   // Red is around 0° or 360°
        bool detectedBlue = (hue >= 180 && hue <= 260); // Blue is around 220°

        if (detectedRed && !isRedTeam) {
            intake.move_velocity(127);  // Flick off red ring if on blue team
            pros::delay(200);
            intake.move_velocity(0);
        } 
        else if (detectedBlue && isRedTeam) {
            intake.move_velocity(127);  // Flick off blue ring if on red team
            pros::delay(200);
            intake.move_velocity(0);
        }

        pros::delay(50);  // Small delay for loop stability
    }
}

// Function to check and update team selection
void checkTeamSelection() {
  while (true) {
      if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
          isRedTeam = true;
          master.print(0, 0, "rojo");
      } 
      else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
          isRedTeam = false;
          master.print(0, 0, "azul");
      }

      pros::delay(10);  // Small delay to prevent spam input
  }
}

void opcontrol() {
  // This is preference to what I like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  pros::Task sortingTask(sortRings);  // Run sorting in a separate task
  pros::Task teamSelectionTask(checkTeamSelection);  // Run team selection

  while (true) {
    ez_template_extras();
    chassis.opcontrol_arcade_standard(ez::SPLIT); 
    // Clamp
    mogo.button_toggle(master.get_digital(DIGITAL_B));
    // intake
    if (master.get_digital(DIGITAL_L1)) {
      intake.move(127);
    } 
    else if (master.get_digital(DIGITAL_L2)) {
      intake.move(-127);
    } 
    else {
      intake.move(0);
    }
    // Doinker
    doinker.button_toggle(master.get_digital(DIGITAL_LEFT));
    // Lady Brown
    if (master.get_digital(DIGITAL_R1)) {
      lady.move(127);
    } 
    else if (master.get_digital(DIGITAL_R2)) {
      lady.move(-127);
    } 
    else {
      lady.move(0);
    }
    // Move Lady Brown To Point
    if (master.get_digital(DIGITAL_UP)) {
      lady.move_absolute(1000, 100); // Move to 1000 encoder ticks at 100% velocity
    }

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
