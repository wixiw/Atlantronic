#ifndef _INIT_H_
#define _INIT_H_

//! @file init.h
//! @brief Contient l'ordre d'initialisation des modules s'ils sont présent et l'ordre de fermeture
//! @author Jean-Baptiste Trédez
//!
//! il s'agit d'un tri par ordre alphabétique (donc "01" < "02" < "1" < "10" < "2" )
//! réalisé à l'édition des liens.
//! pour faire simple et lisible, on n'utilise que les chiffres et
//! on préfixe de "0" : "00" < "01" < "09" < "10" < "19" < "20".
//! en cas d'égalité, l'ordre d'initialisation entre ces fonctions est
//! "la première trouvée par l'éditeur de liens sera la première exécutée".
//!
//! pour la fermeture, le tri est identique mais l'exécution se fait en ordre inverse
//! (plus la priorité de fermeture est grande, plus c'est exécuté tôt). Ceci permet
//! de garder les mêmes numéros sauf exception
//! les #define sont ordonnés pour indiquer le sens d'exécution (croissant pour l'init, décroissant pour la destruction)

#define INIT_TEMPS                    "00"
#define INIT_LOG                      "01"
//#define INIT_VFS                      "02"
//#define INIT_USB                      "03"
//#define INIT_I2C                      "04"
//#define INIT_CAN                      "05"

#define ERR_INIT_LOG               -1

#endif
