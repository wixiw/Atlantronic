#ifndef USART_H
#define USART_H

//! @file usart.h
//! @brief Usart
//! @author Atlantronic

#include "kernel/cpu/cpu.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"

#define USART_MAX_DEVICE      2

enum usart_id
{
	USART3_FULL_DUPLEX,
	UART4_HALF_DUPLEX
};

enum usart_err_codes
{
	ERR_USART_READ_SR_FE,
	ERR_USART_READ_SR_NE,
	ERR_USART_READ_SR_ORE,
	ERR_USART_TIMEOUT,
};

void usart_open( enum usart_id id, uint32_t frequency);

void usart_set_read_dma_buffer(enum usart_id id, unsigned char* buf);
void usart_set_read_dma_size(enum usart_id id, uint16_t size);

//! @return 0 si tout va bien, code d'erreur sinon :
//! ERR_USART_TIMEOUT     : timeout
//! ERR_USART_READ_SR_FE  : desynchro, bruit ou octet "break"
//! ERR_USART_READ_SR_NE  : bruit
//! ERR_USART_READ_SR_ORE : overrun
uint32_t usart_wait_read(enum usart_id id, portTickType timeout);

void usart_set_write_dma_buffer(enum usart_id id, unsigned char* buf);
void usart_send_dma_buffer(enum usart_id id, uint16_t size);

void usart_set_frequency(enum usart_id id, uint32_t frequency);

#endif
