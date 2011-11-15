/*
 * CanARDDictionnaryAccessor.cpp
 *
 *  Created on: 29 avr. 2011
 *      Author: wla
 */

#include "CanARDDictionnaryAccessor.hpp"
#include "orocos/can/dictionnary/CanARD.h"
#include <iostream>

using namespace std;
using namespace arp_hml;

#define IF_ACCESSOR_CASE(name) \
if( concataned == #name )\
{\
	return &name;\
}

CanARDDictionnaryAccessor::CanARDDictionnaryAccessor()
{

}

UNS8* CanARDDictionnaryAccessor::getUNS8Pointer(string componentName, string varName)
{
	string concataned = componentName;
	concataned.append("_");
	concataned.append(varName);

//	cerr << "########################### " << endl;
//	cerr << " access to " << concataned << endl;
//	cerr << "########################### " << endl;

	IF_ACCESSOR_CASE(WoodheadIn_outputs);
	IF_ACCESSOR_CASE(WoodheadOut_outputs);

	IF_ACCESSOR_CASE(LeftSteering_FaulhaberCommandReturn);
	IF_ACCESSOR_CASE(LeftSteering_FaulhaberCommandReturnCode);
	IF_ACCESSOR_CASE(LeftSteering_FaulhaberCommand);

    IF_ACCESSOR_CASE(RightSteering_FaulhaberCommandReturn);
    IF_ACCESSOR_CASE(RightSteering_FaulhaberCommandReturnCode);
    IF_ACCESSOR_CASE(RightSteering_FaulhaberCommand);

    IF_ACCESSOR_CASE(RearSteering_FaulhaberCommandReturn);
    IF_ACCESSOR_CASE(RearSteering_FaulhaberCommandReturnCode);
    IF_ACCESSOR_CASE(RearSteering_FaulhaberCommand);

	IF_ACCESSOR_CASE(LeftDriving_FaulhaberCommandReturn);
	IF_ACCESSOR_CASE(LeftDriving_FaulhaberCommandReturnCode);
	IF_ACCESSOR_CASE(LeftDriving_FaulhaberCommand);

	IF_ACCESSOR_CASE(RightDriving_FaulhaberCommandReturn);
	IF_ACCESSOR_CASE(RightDriving_FaulhaberCommandReturnCode);
	IF_ACCESSOR_CASE(RightDriving_FaulhaberCommand);

    IF_ACCESSOR_CASE(RearDriving_FaulhaberCommandReturn);
    IF_ACCESSOR_CASE(RearDriving_FaulhaberCommandReturnCode);
    IF_ACCESSOR_CASE(RearDriving_FaulhaberCommand);

	return NULL;
}

UNS16* CanARDDictionnaryAccessor::getUNS16Pointer(string componentName, string varName)
{
	string concataned = componentName;
	concataned.append("_");
	concataned.append(varName);

//	cerr << "########################### " << endl;
//	cerr << " access to " << concataned << endl;
//	cerr << "########################### " << endl;

    IF_ACCESSOR_CASE(LeftSteering_Ds402State);
    IF_ACCESSOR_CASE(RightSteering_Ds402State);
    IF_ACCESSOR_CASE(RearSteering_Ds402State);
	IF_ACCESSOR_CASE(LeftDriving_Ds402State);
	IF_ACCESSOR_CASE(RightDriving_Ds402State);
	IF_ACCESSOR_CASE(RearDriving_Ds402State);

	return NULL;
}


UNS32* CanARDDictionnaryAccessor::getUNS32Pointer(string componentName, string varName)
{
	string concataned = componentName;
	concataned.append("_");
	concataned.append(varName);

//	cerr << "########################### " << endl;
//	cerr << " access to " << concataned << endl;
//	cerr << "########################### " << endl;

    IF_ACCESSOR_CASE(LeftSteering_FaulhaberCommandParameter);
    IF_ACCESSOR_CASE(LeftSteering_FaulhaberCommandReturnParameter);
    IF_ACCESSOR_CASE(RightSteering_FaulhaberCommandParameter);
    IF_ACCESSOR_CASE(RightSteering_FaulhaberCommandReturnParameter);
    IF_ACCESSOR_CASE(RearSteering_FaulhaberCommandParameter);
    IF_ACCESSOR_CASE(RearSteering_FaulhaberCommandReturnParameter);
	IF_ACCESSOR_CASE(LeftDriving_FaulhaberCommandParameter);
	IF_ACCESSOR_CASE(LeftDriving_FaulhaberCommandReturnParameter);
	IF_ACCESSOR_CASE(RightDriving_FaulhaberCommandParameter);
	IF_ACCESSOR_CASE(RightDriving_FaulhaberCommandReturnParameter);
    IF_ACCESSOR_CASE(RearDriving_FaulhaberCommandParameter);
    IF_ACCESSOR_CASE(RearDriving_FaulhaberCommandReturnParameter);

	return NULL;
}

INTEGER16* CanARDDictionnaryAccessor::getINTEGER16Pointer(string componentName, string varName)
{
	string concataned = componentName;
	concataned.append("_");
	concataned.append(varName);

//	cerr << "########################### " << endl;
//	cerr << " access to " << concataned << endl;
//	cerr << "########################### " << endl;

    IF_ACCESSOR_CASE(LeftSteering_MeasuredCurrent);
    IF_ACCESSOR_CASE(RightSteering_MeasuredCurrent);
    IF_ACCESSOR_CASE(RearSteering_MeasuredCurrent);
	IF_ACCESSOR_CASE(LeftDriving_MeasuredCurrent);
	IF_ACCESSOR_CASE(RightDriving_MeasuredCurrent);
	IF_ACCESSOR_CASE(RearDriving_MeasuredCurrent);

	return NULL;
}

INTEGER32* CanARDDictionnaryAccessor::getINTEGER32Pointer(string componentName, string varName)
{
	string concataned = componentName;
	concataned.append("_");
	concataned.append(varName);

//	cerr << "########################### " << endl;
//	cerr << " access to " << concataned << endl;
//	cerr << "########################### " << endl;

    IF_ACCESSOR_CASE(LeftSteering_MeasuredPosition);
    IF_ACCESSOR_CASE(RightSteering_MeasuredPosition);
    IF_ACCESSOR_CASE(RearSteering_MeasuredPosition);
	IF_ACCESSOR_CASE(LeftDriving_MeasuredPosition);
	IF_ACCESSOR_CASE(RightDriving_MeasuredPosition);
	IF_ACCESSOR_CASE(RearDriving_MeasuredPosition);

	return NULL;
}
