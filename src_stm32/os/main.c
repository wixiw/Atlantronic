//! @file main.c
//! @brief Programme principal
//! @author Atlantronic

#include <stdint.h>
#include "error_hook.h"
#include "module.h"
#include "os.h"

//! pour ne pas confondre avec le main de la libc newlib
void __main() __attribute__((noreturn));

void __main()
{
	uint8_t err = initModules();

	// on n'arrive normalement jamais ici sur la cible
	kernel_panic(err);
}

