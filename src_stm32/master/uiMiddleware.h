/*
 * uiMiddleware.hpp
 *
 *  Created on: Feb 25, 2015
 *      Author: willy
 *
 */

#ifndef UIMIDDLEWARE_HPP_
#define UIMIDDLEWARE_HPP_

#include <stdint.h>
#include "components/robot/color.h"

/*
 *  The UiMiddleware is an aggregation class to decouple the information to send to the user
 *  and their implementation (HW choice : LED, sound, ...)
 */

//
// Inform : Robot => User
//

/** Inform the user that we would like him to withdraw the start */
void ui_ubiquityWaitingStartIn();

/** Inform the user that the robot is booting */
void ui_ubiquityBooting();

/** Inform the user that the robot is ready for the match */
void ui_ubiquityReadyForMatch();

/** Inform the user that the robot is waiting match start */
void ui_ubiquityWaitForMatch();

/** Inform the user that the robot is ready to begin its self tests*/
void ui_ubiquityReadyForSelfTests();

/** Inform the user that the robot is self testing*/
void ui_selfTesting();

/** Inform the user that the match is running */
void ui_matchRuning();

/** Inform the user that the match is running */
void ui_matchEnded();

/** Inform the user that the Emergency Stop is active */
void ui_displayEmergencyStopActive();

/** Inform the user that the heartbeat has not been received for long */
void ui_heartbeatLost();

//
// Requests : Robot => User => Robot
//

/** Request the user to choose the color, blocking call */
eMatchColor ui_requestMatchColor();

/** Blocking call that waits for the start to be withdrawn */
void ui_waitForMatchStart();

/** Blocking call that waits for the selfTest beginning */
void ui_waitForSelfTestStart();

/** Read start position */
bool ui_isStartPlugged();



/** Inform the robot that the Debug 1 event is sent (generated by the color button) */
//extern void ui_user1_event();

/** Inform the robot that the Debug 2 event is sent (not mapped yet) */
//extern void ui_user2_event();

#endif /* UIMIDDLEWARE_HPP_ */