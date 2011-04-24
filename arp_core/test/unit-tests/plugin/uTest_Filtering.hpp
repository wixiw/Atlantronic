/*
 * test_Filtering.cpp
 *
 *  Created on: 12 sept. 2010
 *      Author: willy
 */

#include "plugin/FilteringService.cpp"

FilteringService f(NULL);
double period = 2.0;//s

BOOST_AUTO_TEST_CASE( Filtering_FirstDerivative )
{
    /*
     * cas d'erreur
     */
    //tous arguments nulls
    BOOST_CHECK_EQUAL( f.firstDerivateLimitation(0,0,0,0,0),0 );
    BOOST_CHECK_EQUAL( f.firstDerivateLimitation(0,13,0,0,0),13 );
    BOOST_CHECK_EQUAL( f.firstDerivateLimitation(0,13,-1,0,0),13 );
    BOOST_CHECK_EQUAL( f.firstDerivateLimitation(1,13,0,-1,1),13 );

    //vmin plus grand que vmax
    BOOST_CHECK_EQUAL( f.firstDerivateLimitation(0,13,period,1,-1),13 );

    //vmin=vmax=0
    BOOST_CHECK_EQUAL( f.firstDerivateLimitation(1,21,period,0,0),21 );
    BOOST_CHECK_EQUAL( f.firstDerivateLimitation(-1,21,period,0,0),21 );

    /*
     * cas normaux
     */
    //entrée positive : sortie = entrée avec bornes symétriques "infinies"
    BOOST_CHECK_EQUAL( f.firstDerivateLimitation(7,2,period,-10000,10000),7 );
    //entrée positive : sortie = entrée avec borne inf positive et sup "infinie"
    BOOST_CHECK_EQUAL( f.firstDerivateLimitation(7,2,period,1,10000),7 );
    //entrée négative : sortie = entrée avec bornes symétriques "infinies"
    BOOST_CHECK_EQUAL( f.firstDerivateLimitation(-7,2,period,-10000,10000),-7 );
    //entrée négative : sortie = entrée avec borne inf "infinie" et sup négative
    BOOST_CHECK_EQUAL( f.firstDerivateLimitation(-7,2,period,-10000,1),-7 );

    //entrée positive : sortie = entrée avec bornes symétriques "infinies" + changement de période
    BOOST_CHECK_EQUAL( f.firstDerivateLimitation(7,2,period/1.5,-10000,10000),7 );
    //entrée positive : sortie = entrée avec borne inf positive et sup "infinie" + changement de période
    BOOST_CHECK_EQUAL( f.firstDerivateLimitation(7,2,period/1.5,1,10000),7 );
    //entrée négative : sortie = entrée avec bornes symétriques "infinies" + changement de période
    BOOST_CHECK_EQUAL( f.firstDerivateLimitation(-7,2,period/1.5,-10000,10000),-7 );
    //entrée négative : sortie = entrée avec borne inf "infinie" et sup négative + changement de période
    BOOST_CHECK_EQUAL( f.firstDerivateLimitation(-7,2,period/1.5,-10000,1),-7 );

    /*
     * cas saturés
     */
    //entrée infinie positive : sortie limitée par la borne sup (borne inf normale)
    BOOST_CHECK_EQUAL( f.firstDerivateLimitation(10000,1,period,-3,4),9 );
    //entrée infinie positive : sortie limitée par la borne sup (borne inf positive)
    BOOST_CHECK_EQUAL( f.firstDerivateLimitation(10000,1,period,1,4),9 );
    //entrée infinie positive : sortie limitée par la borne sup (borne inf "infinie")
    BOOST_CHECK_EQUAL( f.firstDerivateLimitation(10000,1,period,-10000,4),9 );
    //entrée infinie positive : sortie limitée par la borne sup (borne inf normale) + changement de période
    BOOST_CHECK_EQUAL( f.firstDerivateLimitation(10000,1,period/1.5,-3,4),1+8.0/1.5 );

    //entrée infinie négative : sortie limitée par la borne sup (borne inf normale)
    BOOST_CHECK_EQUAL( f.firstDerivateLimitation(-10000,1,period,-3,4),-5 );
    //entrée infinie négative : sortie limitée par la borne sup (borne inf positive)
    BOOST_CHECK_EQUAL( f.firstDerivateLimitation(-10000,1,period,-3,1),-5 );
    //entrée infinie négative : sortie limitée par la borne sup (borne inf "infinie")
    BOOST_CHECK_EQUAL( f.firstDerivateLimitation(-10000,1,period,-3,10000),-5 );
    //entrée infinie négative : sortie limitée par la borne sup (borne inf normale)
    BOOST_CHECK_EQUAL( f.firstDerivateLimitation(-10000,1,period/1.5,-3,4),-6.0/1.5+1 );
}
