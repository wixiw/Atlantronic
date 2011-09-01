/*
 * TestJoystick.cpp
 *
 *  Created on: 13 may 2011
 *      Author: wla
 */



#include "TestJoystick.hpp"
#include <rtt/Component.hpp>
#include "orocos/taskcontexts/HmlTaskContext.hpp"

#define BOOST_TEST_DYN_LINK
#include <boost/test/included/unit_test.hpp>
#include <boost/bind.hpp>
using namespace boost;
using namespace boost::unit_test;
using namespace boost::unit_test::framework;

using namespace arp_hml;
using namespace arp_core;

TestJoystick* tcTest;
TaskContext* tcJoystick;

TestJoystick::TestJoystick(const std::string& name):
	HmlTaskContext(name)
{
	long attrTcTest = (long) tcTest;
	long attrTcJoystick = (long) tcJoystick;
	addAttribute("tcTest", attrTcTest);
	addAttribute("tcJoystick", attrTcJoystick);

	addPort("inButton",inButton);
	addPort("inAxe",inAxe);

}

bool TestJoystick::configureHook()
{
	bool res = true;

	tcTest = this;
	tcJoystick = getPeer("Joystick");


	if( tcTest == NULL || tcJoystick == NULL)
	{
		LOG(Error) << "TaskContext pointers not preperly initialised" << endlog();
		LOG(Error) << "tcTest : " << tcTest << " tcJoystick : " << tcJoystick << endlog();
		res = false;
	}
	return res;
}

bool TestJoystick::startHook()
{
	//bool res = HmlTaskContext::startHook();
	return true;
}

void checkConnectivity()
{
	cout << "**********************************************" << endl;
	cout << "Testing connectivity ... " << endl;
	if( tcJoystick->isRunning() && !tcJoystick->inRunTimeError() )
	{
		BOOST_CHECK(true);
		cout << "Joystick properly connected" << endl;
	}
	else
	{
		cout << "The joystick is not connected ! Aborting..." << endl;
		BOOST_CHECK(false);
		return;
	}

	cout << "Please disconnect the joystick" << endl;
	while( !tcJoystick->inRunTimeError() ){ sleep(1);}
	BOOST_CHECK( tcJoystick->isRunning() && tcJoystick->inRunTimeError() );
	Attribute<bool> a = tcJoystick->getAttribute("attrIsConnected");
	if(a.get()==false)
	{
		BOOST_CHECK( true );
		cout << "Joystick properly disconnected" << endl;
	}
	else
	{
		cout << "The joystick is not well disconnected ! Aborting..." << endl;
		BOOST_CHECK( false );
		return;
	}

	cout << "Please reconnect a wrong joystick" << endl;
	a = tcJoystick->getAttribute("attrIsConnected");
	while( !a.get() ){ sleep(1);}
	BOOST_CHECK( tcJoystick->isRunning() && tcJoystick->inRunTimeError() );
	if(a.get()==true)
	{
		BOOST_CHECK( true );
		cout << "Joystick properly connected" << endl;
	}
	else
	{
		cout << "The (voluntary wrong) joystick is not well reconnected ! Aborting..." << endl;
		BOOST_CHECK( false );
		return;
	}
	a = tcJoystick->getAttribute("attrIsIdentityOk");
	if(a.get()==false)
	{
		BOOST_CHECK( true );
		cout << "Joystick has detected the wrong hardware" << endl;
	}
	else
	{
		cout << "The (voluntary wrong) joystick is not detected ! Aborting..." << endl;
		BOOST_CHECK( false );
		return;
	}

	cout << "Please disconnect the joystick" << endl;
	a = tcJoystick->getAttribute("attrIsConnected");
	while( a.get() ){ sleep(1);}
	BOOST_CHECK( tcJoystick->isRunning() && tcJoystick->inRunTimeError() );
	if(a.get()==false)
	{
		BOOST_CHECK( true );
		cout << "Joystick properly disconnected" << endl;
	}
	else
	{
		cout << "The joystick is not well disconnected ! Aborting..." << endl;
		BOOST_CHECK( false );
		return;
	}

	cout << "Please connect the right joystick" << endl;
	while( tcJoystick->inRunTimeError() ){ sleep(1);}
	BOOST_CHECK( tcJoystick->isRunning() && !tcJoystick->inRunTimeError() );
	a = tcJoystick->getAttribute("attrIsConnected");
	if(a.get()==true)
	{
		BOOST_CHECK( true );
		cout << "Joystick properly connected" << endl;
	}
	else
	{
		cout << "The joystick is not well reconnected ! Aborting..." << endl;
		BOOST_CHECK( false );
		return;
	}

}

void CHECK_BUTTON(string button_name)
{
	bool buttonValue = false;
	tcTest->inButton.disconnect();
	tcTest->inButton.connectTo(tcJoystick->getPort(button_name));
	cout << "please press " << button_name << endl;
	while( buttonValue==false )
	{
		tcTest->inButton.readNewest(buttonValue);
	}
	cout << "Button " << button_name << " pressed OK" << endl;
	BOOST_CHECK( buttonValue );
}

void checkButtons()
{
	cout << "**********************************************" << endl;
	cout << "Checking buttons..." << endl;
	CHECK_BUTTON("outButton1");
	CHECK_BUTTON("outButton2");
	CHECK_BUTTON("outButton3");
	CHECK_BUTTON("outButton4");
	CHECK_BUTTON("outTrigger5");
	CHECK_BUTTON("outTrigger6");
	CHECK_BUTTON("outTrigger7");
	CHECK_BUTTON("outTrigger8");
	CHECK_BUTTON("outButton9");
	CHECK_BUTTON("outButton10");
}


void CHECK_AXE(string axe_name)
{
	double axis_value = 6;
	tcTest->inAxe.disconnect();
	tcTest->inAxe.connectTo(tcJoystick->getPort(axe_name));

	while( axis_value > -0.9 )
	{
		tcTest->inAxe.readNewest(axis_value);
	}
	cout << "Low value of axe " << axe_name << " pushed OK" << endl;
	BOOST_CHECK( true );

	axis_value = -6;
	while( axis_value < 0.9 )
	{
		tcTest->inAxe.readNewest(axis_value);
	}
	cout << "High value of axe " << axe_name << " pushed OK" << endl;
	BOOST_CHECK( true );
}

void checkAxes()
{
	cout << "**********************************************" << endl;
	cout << "Checking axes ..." << endl;
	cout << "please push left directional cross button, then right " << endl;
	CHECK_AXE("outPadX");
	cout << "please push down directional cross button, then up " << endl;
	CHECK_AXE("outPadY");
	cout << "please push left joystick to left, then right " << endl;
	CHECK_AXE("outX1");
	cout << "please push left joystick to up, then down " << endl;
	CHECK_AXE("outY1");
	cout << "please push right joystick to left, then right " << endl;
	CHECK_AXE("outX2");
	cout << "please push right joystick to up, then down " << endl;
	CHECK_AXE("outY2");
}

void checkTimeout()
{
	cout << "**********************************************" << endl;
	cout << "checking Timeout..." << endl;
	cout << "Please Push the left Joystick up and maintain it withou moving " << endl;
	double axis_value = 6;
	tcTest->inAxe.disconnect();
	tcTest->inAxe.connectTo(tcJoystick->getPort("outY1"));

	while( axis_value > -0.9 )
	{
		tcTest->inAxe.readNewest(axis_value);
	}
	Property<double> p = tcJoystick->getProperty("propMaxNoEventDelay");
	cout << "Pushed up OK, wait " << p << " s"<< endl;
	usleep((int)(p*1E6*1.05));
	tcTest->inAxe.readNewest(axis_value);
	BOOST_CHECK( axis_value==0.0 );
}

bool init_function()
{
    /** nom de la suite de test principale */
    master_test_suite().p_name.value = "Joystick Test Suite";
    master_test_suite().add( BOOST_TEST_CASE( ::bind( &checkConnectivity) ) );
    master_test_suite().add( BOOST_TEST_CASE( ::bind( &checkButtons) ) );
    master_test_suite().add( BOOST_TEST_CASE( ::bind( &checkAxes) ) );
    master_test_suite().add( BOOST_TEST_CASE( ::bind( &checkTimeout) ) );

    return true;
}

void TestJoystick::updateHook()
{
    //appel du parent car il log les bootUp
	//HmlTaskContext::updateHook();

    int argc = 0;
    char* argv;
    argv = (char*)malloc(10);
    unit_test_main( &init_function, argc, &argv );
}



ORO_LIST_COMPONENT_TYPE( arp_hml::TestJoystick )
