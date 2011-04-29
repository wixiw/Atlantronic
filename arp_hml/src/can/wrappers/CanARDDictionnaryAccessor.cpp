/*
 * CanARDDictionnaryAccessor.cpp
 *
 *  Created on: 29 avr. 2011
 *      Author: wla
 */

#include "CanARDDictionnaryAccessor.hpp"
#include "can/dictionnary/CanARD.h"
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

	IF_ACCESSOR_CASE(Woodhead_outputs);
	IF_ACCESSOR_CASE(FrontSteering_FaulhaberCommandReturn);
	IF_ACCESSOR_CASE(FrontSteering_FaulhaberCommandReturnCode);
	IF_ACCESSOR_CASE(LeftDriving_FaulhaberCommandReturn);
	IF_ACCESSOR_CASE(LeftDriving_FaulhaberCommandReturnCode);
	IF_ACCESSOR_CASE(RightDriving_FaulhaberCommandReturn);
	IF_ACCESSOR_CASE(RightDriving_FaulhaberCommandReturnCode);
	IF_ACCESSOR_CASE(LeftDriving_FaulhaberCommand);
	IF_ACCESSOR_CASE(RightDriving_FaulhaberCommand);

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

	IF_ACCESSOR_CASE(FrontSteering_Ds402State);
	IF_ACCESSOR_CASE(LeftDriving_Ds402State);
	IF_ACCESSOR_CASE(RightDriving_Ds402State);

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

	IF_ACCESSOR_CASE(FrontSteering_FaulhaberCommandParameter);
	IF_ACCESSOR_CASE(FrontSteering_FaulhaberCommandReturnParameter);
	IF_ACCESSOR_CASE(LeftDriving_FaulhaberCommandParameter);
	IF_ACCESSOR_CASE(LeftDriving_FaulhaberCommandReturnParameter);
	IF_ACCESSOR_CASE(RightDriving_FaulhaberCommandParameter);
	IF_ACCESSOR_CASE(RightDriving_FaulhaberCommandReturnParameter);

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

	IF_ACCESSOR_CASE(FrontSteering_MeasuredCurrent);
	IF_ACCESSOR_CASE(LeftDriving_MeasuredCurrent);
	IF_ACCESSOR_CASE(RightDriving_MeasuredCurrent);

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

	IF_ACCESSOR_CASE(FrontSteering_MeasuredPosition);
	IF_ACCESSOR_CASE(LeftDriving_MeasuredPosition);
	IF_ACCESSOR_CASE(RightDriving_MeasuredPosition);


	return NULL;
}
