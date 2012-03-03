/*
 * JsonScanParser.cpp
 *
 *  Created on: 2 mars 2012
 *      Author: Boris
 *
 */

#include "JsonScanParser.hpp"

#include "LSL/Logger.hpp"

using namespace arp_rlu;
using namespace lsl;
using namespace vjson;
using namespace arp_core::log;

JsonScanParser::JsonScanParser()
: JsonDocument()
{
}

JsonScanParser::JsonScanParser(std::string filename)
: JsonDocument()
{
    this->parse(filename.c_str());
}

bool JsonScanParser::parse(const char *filename)
{
    return JsonDocument::parse(filename);
}

bool JsonScanParser::getScan(LaserScan & ls)
{
    json_value * r = JsonDocument::root();
    if(!r)
    {
        Log( ERROR ) << "JsonScanParser::getScan" << " - " << "root is NULL";
        return false;
    }

    std::vector< std::string > childs = JsonDocument::getChildNames(r);
    if(childs.size() != 5)
    {
        Log( ERROR ) << "JsonScanParser::getScan" << " - " << "childs.size() != 5  (childs.size() = " << childs.size() << ")";
        return false;
    }

    json_value * typeChild = JsonDocument::getChild(r, "type");
    if(!typeChild)
    {
        Log( ERROR ) << "JsonScanParser::getScan" << " - " << "No \"type\" child";
        return false;
    }

    std::string typeName = JsonDocument::getStringData(typeChild);
    if( typeName.compare("Scan") != 0 )
    {
        Log( ERROR ) << "JsonScanParser::getScan" << " - " << "Type is not \"Scan\"";
        return false;
    }

    json_value * sizeChild = JsonDocument::getChild(r, "size");
    if(!sizeChild)
    {
        Log( ERROR ) << "JsonScanParser::getScan" << " - " << "No \"size\" child";
        return false;
    }

    json_value * ttChild = JsonDocument::getChild(r, "tt");
    if(!ttChild)
    {
        Log( ERROR ) << "JsonScanParser::getScan" << " - " << "No \"tt\" child";
        return false;
    }

    json_value * rangeChild = JsonDocument::getChild(r, "range");
    if(!rangeChild)
    {
        Log( ERROR ) << "JsonScanParser::getScan" << " - " << "No \"range\" child";
        return false;
    }

    json_value * thetaChild = JsonDocument::getChild(r, "theta");
    if(!thetaChild)
    {
        Log( ERROR ) << "JsonScanParser::getScan" << " - " << "No \"theta\" child";
        return false;
    }

    unsigned int size = (unsigned int)JsonDocument::getIntegerData(sizeChild);

    std::vector< json_type > ttType = JsonDocument::getChildTypes( ttChild );
    if( ttType.size() != size )
    {
        Log( ERROR ) << "JsonScanParser::getScan" << " - " << "tt.size() != size";
        return false;
    }

    std::vector< json_type > rangeType = JsonDocument::getChildTypes( rangeChild );
    if( rangeType.size() != size )
    {
        Log( ERROR ) << "JsonScanParser::getScan" << " - " << "range.size() != size";
        return false;
    }

    std::vector< json_type > thetaType = JsonDocument::getChildTypes( thetaChild );
    if( thetaType.size() != size )
    {
        Log( ERROR ) << "JsonScanParser::getScan" << " - " << "theta.size() != size";
        return false;
    }

    for( std::vector< json_type >::iterator it = ttType.begin() ; it != ttType.end() ; ++it )
    {
        if( *it != JSON_FLOAT )
        {
            Log( ERROR ) << "JsonScanParser::getScan" << " - " << "Data in \"tt\" is not all FLOAT";
            return false;
        }
    }

    for( std::vector< json_type >::iterator it = rangeType.begin() ; it != rangeType.end() ; ++it )
    {
        if( *it != JSON_FLOAT )
        {
            Log( ERROR ) << "JsonScanParser::getScan" << " - " << "Data in \"range\" is not all FLOAT";
            return false;
        }
    }

    for( std::vector< json_type >::iterator it = thetaType.begin() ; it != thetaType.end() ; ++it )
    {
        if( *it != JSON_FLOAT )
        {
            Log( ERROR ) << "JsonScanParser::getScan" << " - " << "Data in \"theta\" is not all FLOAT";
            return false;
        }
    }

    Eigen::MatrixXd polar(3,size);
    for(unsigned int i = 0 ; i < size ; i++)
    {
        polar(0,i) = JsonDocument::getFloatData(JsonDocument::getChild( ttChild, i ));
        polar(1,i) = JsonDocument::getFloatData(JsonDocument::getChild( rangeChild, i ));
        polar(2,i) = JsonDocument::getFloatData(JsonDocument::getChild( thetaChild, i ));
    }

    ls.setPolarData(polar);
    return true;
}

