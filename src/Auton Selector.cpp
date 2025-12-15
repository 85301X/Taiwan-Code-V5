#include "liblvgl/llemu.hpp"
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
//#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/colors.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/gps.hpp"
#include "pros/motors.h"
#include "pros/optical.h"
#include "pros/optical.hpp"
#include <cstdio>
#include <string>
#include "global.h"
#include <iostream>

enum class Alliance {
    RED,
    BLUE
};

enum class AutonType {
    SOLO_AWP,
    HIGH_GOAL,
    LOW_GOAL
};


Alliance selectedAlliance = Alliance::RED;
AutonType selectedAuton   = AutonType::SOLO_AWP;
bool autonConfirmed = false;



void drawAllianceScreen() {
    pros::screen::erase();

    switch (selectedAlliance) {
        case Alliance::RED:
           pros::screen::set_pen(pros::Color::red);
    pros::screen::fill_rect(5,5,480,480);
            break;
        case Alliance::BLUE:
              pros::screen::set_pen(pros::Color::blue);
      pros::screen::fill_rect(5,5,480,480);
            break;
    }

    pros::screen::set_pen(pros::Color::white);
    pros::screen::print(pros::E_TEXT_LARGE, 120, 30, "SELECT ALLIANCE");

    pros::screen::print(
        pros::E_TEXT_LARGE,
        180,
        100,
        selectedAlliance == Alliance::RED ? "RED" : "BLUE"
    );

    pros::screen::print(pros::E_TEXT_SMALL, 120, 180, "LEFT: Toggle   A: Confirm");
}

void drawAutonScreen() {
    pros::screen::erase();

  
    switch (selectedAlliance) {
        case Alliance::RED:
           pros::screen::set_pen(pros::Color::red);
    pros::screen::fill_rect(5,5,480,480);
            break;
        case Alliance::BLUE:
              pros::screen::set_pen(pros::Color::blue);
      pros::screen::fill_rect(5,5,480,480);
            break;
    }

      pros::screen::set_pen(pros::Color::white);

    pros::screen::print(pros::E_TEXT_LARGE, 140, 30, "SELECT AUTON");

    const char* autonName;

    switch (selectedAuton) {
        case AutonType::SOLO_AWP:
            autonName = "SOLO AWP";
            break;
        case AutonType::HIGH_GOAL:
            autonName = "HIGH GOAL";
            break;
        case AutonType::LOW_GOAL:
            autonName = "LOW GOAL";
            break;
    }

    pros::screen::print(pros::E_TEXT_LARGE, 160, 100, autonName);
    pros::screen::print(pros::E_TEXT_SMALL, 120, 180, "LEFT: Cycle   A: Confirm");
}

void drawConfirmedScreen() {
    pros::screen::erase();
     switch (selectedAlliance) {
        case Alliance::RED:
           pros::screen::set_pen(pros::Color::red);
    pros::screen::fill_rect(5,5,480,480);
            break;
        case Alliance::BLUE:
              pros::screen::set_pen(pros::Color::blue);
      pros::screen::fill_rect(5,5,480,480);
            break;
    }

      pros::screen::set_pen(pros::Color::white);

    pros::screen::print(pros::E_TEXT_LARGE, 120, 80, "AUTON LOCKED");

    pros::screen::print(
        pros::E_TEXT_MEDIUM,
        140,
        130,
        selectedAlliance == Alliance::RED ? "RED" : "BLUE"
    );

    const char* autonName;

    switch (selectedAuton) {
        case AutonType::SOLO_AWP:  autonName = "SOLO AWP";  break;
        case AutonType::HIGH_GOAL: autonName = "HIGH GOAL"; break;
        case AutonType::LOW_GOAL:  autonName = "LOW GOAL";  break;
    }

    pros::screen::print(pros::E_TEXT_MEDIUM, 140, 160, autonName);
}



void autonSelector() {
    enum class MenuState {
        SELECT_ALLIANCE,
        SELECT_AUTON,
        CONFIRMED
    };

    MenuState state = MenuState::SELECT_ALLIANCE;

    drawAllianceScreen();

    while (!autonConfirmed && pros::competition::is_disabled()) {

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            switch (state) {

                case MenuState::SELECT_ALLIANCE:
                    selectedAlliance =
                        (selectedAlliance == Alliance::RED) ? Alliance::BLUE : Alliance::RED;
                    drawAllianceScreen();
                    break;

                case MenuState::SELECT_AUTON:
                    selectedAuton =
                        static_cast<AutonType>((static_cast<int>(selectedAuton) + 1) % 3);
                    drawAutonScreen();
                    break;

                default:
                    break;
            }
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            switch (state) {

                case MenuState::SELECT_ALLIANCE:
                    state = MenuState::SELECT_AUTON;
                    drawAutonScreen();
                    break;

                case MenuState::SELECT_AUTON:
                    autonConfirmed = true;
                    state = MenuState::CONFIRMED;
                    drawConfirmedScreen();
                    break;

                default:
                    break;
            }
        }

        pros::delay(20);
    }
}
