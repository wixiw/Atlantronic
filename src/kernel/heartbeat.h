#ifndef HEARTBEAT_H
#define HEARTBEAT_H

//! @file heartbeat.h
//! @brief Gestion heartbeat
//! @author Atlantronic

#ifdef __cplusplus
extern "C" {
#endif

void heartbeat_kick();
void heartbeat_enable();
void heartbeat_update();


#ifdef __cplusplus
}
#endif

#endif
