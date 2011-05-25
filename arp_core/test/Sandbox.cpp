/*
 * Sandbox.cpp
 *
 *  Created on: 30 sept. 2010
 *      Author: ard
 */

#include "Sandbox.hpp"
#include <misc/Bite.hpp>
#include <iostream>
#include "version_Core.h"

#include <rtt/os/startstop.h>
#include <log4cpp/HierarchyMaintainer.hh>
#include <log4cpp/Category.hh>
#include <log4cpp/FileAppender.hh>
#include <log4cpp/PatternLayout.hh>
#include <log4cpp/PropertyConfigurator.hh>

using namespace std;
using namespace arp_core;

namespace CoreTest
{

    Sandbox::Sandbox()
    {

    }

    Sandbox::~Sandbox()
    {
    }

}


/* Fonction principale. */
int main(int n, char *params[])
{
    int i;

    /* Affiche le nom du programme : */
    cout << "Nom du programme : " << params[0] << endl;

    /* Affiche la ligne de commande : */
    for (i=1; i<n; ++i)
        cout << "Argument " << i << " : " << params[i] << endl;


    cout << "APR-Core version : "
            << ARP_CORE_VERSION_MAJOR << "."
            << ARP_CORE_VERSION_MINOR << "."
            << ARP_CORE_VERSION_PATCH << endl;

    log4cpp::HierarchyMaintainer::set_category_factory(
        OCL::logging::Category::createOCLCategory);
    //__os_init(0,NULL);


    Bite b1("poil");
    b1.coPrintBite();
    cout << "etat de la bite : " << b1.getTaskState() << endl;





    cout << "configuration " << b1.configure()<< endl;
    cout << "etat de la bite : " << b1.getTaskState() << endl;

    return 0;



}
