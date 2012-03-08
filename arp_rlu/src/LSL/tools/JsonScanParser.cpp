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
    if(!this->parse(filename.c_str()))
        Log( ERROR ) << "JsonScanParser::JsonScanParser" << " - " << "parsing " << filename << " failed";
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
    if(!(childs.size() == 5 || childs.size() == 8))
    {
        Log( ERROR ) << "JsonScanParser::getScan" << " - " << "childs.size() != (5 or 8)  (childs.size() = " << childs.size() << ")";
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

    json_value * xxChild = JsonDocument::getChild(r, "xx");
    json_value * yyChild = JsonDocument::getChild(r, "yy");
    json_value * hhChild = JsonDocument::getChild(r, "hh");
    if(xxChild && yyChild && hhChild)
    {
        std::vector< json_type > xxType = JsonDocument::getChildTypes( xxChild );
        if( xxType.size() != size )
        {
            Log( ERROR ) << "JsonScanParser::getScan" << " - " << "xx.size() != size";
            return false;
        }
        std::vector< json_type > yyType = JsonDocument::getChildTypes( yyChild );
        if( yyType.size() != size )
        {
            Log( ERROR ) << "JsonScanParser::getScan" << " - " << "yy.size() != size";
            return false;
        }
        std::vector< json_type > hhType = JsonDocument::getChildTypes( hhChild );
        if( hhType.size() != size )
        {
            Log( ERROR ) << "JsonScanParser::getScan" << " - " << "hh.size() != size";
            return false;
        }
        for( std::vector< json_type >::iterator it = xxType.begin() ; it != xxType.end() ; ++it )
        {
            if( *it != JSON_FLOAT )
            {
                Log( ERROR ) << "JsonScanParser::getScan" << " - " << "Data in \"xx\" is not all FLOAT";
                return false;
            }
        }
        for( std::vector< json_type >::iterator it = yyType.begin() ; it != yyType.end() ; ++it )
        {
            if( *it != JSON_FLOAT )
            {
                Log( ERROR ) << "JsonScanParser::getScan" << " - " << "Data in \"yy\" is not all FLOAT";
                return false;
            }
        }
        for( std::vector< json_type >::iterator it = hhType.begin() ; it != hhType.end() ; ++it )
        {
            if( *it != JSON_FLOAT )
            {
                Log( ERROR ) << "JsonScanParser::getScan" << " - " << "Data in \"hh\" is not all FLOAT";
                return false;
            }
        }
        Eigen::VectorXd xx = Eigen::VectorXd::Zero(size);
        Eigen::VectorXd yy = Eigen::VectorXd::Zero(size);
        Eigen::VectorXd hh = Eigen::VectorXd::Zero(size);
        for(unsigned int i = 0 ; i < size ; i++)
        {
            xx(i) = JsonDocument::getFloatData(JsonDocument::getChild( xxChild, i ));
            yy(i) = JsonDocument::getFloatData(JsonDocument::getChild( yyChild, i ));
            hh(i) = JsonDocument::getFloatData(JsonDocument::getChild( hhChild, i ));
        }
        ls.computeCartesianData(ls.getTimeData(), xx, yy, hh);
    }

    return true;
}


bool arp_rlu::lsl::export_json(const LaserScan & ls, const std::string filename)
{
    std::ofstream file;
    file.open(filename.c_str());

    file << std::fixed;

    file << '{' << std::endl;
    file << "  \"type\": \"Scan\"," << std::endl;
    unsigned int N = ls.getSize();
    if(N == 0)
    {
        file << "  \"tt\" : []," << std::endl;
        file << "  \"range\" : []," << std::endl;
        file << "  \"theta\" : []," << std::endl;
        file << "  \"xx\" : []," << std::endl;
        file << "  \"yy\" : []," << std::endl;
        file << "  \"hh\" : []," << std::endl;
    }
    else
    {
        Eigen::MatrixXd polar = ls.getPolarData();
        file << "  \"tt\": [" << std::endl;
        for(unsigned int i = 0 ; i < N ; i++)
        {
            file << "    " << polar(0,i);
            if( i < N-1 )
                file << ",";
            file << std::endl;
        }
        file << "  ]," << std::endl;

        file << "  \"range\": [" << std::endl;
        for(unsigned int i = 0 ; i < N ; i++)
        {
            file << "    " << (double)polar(1,i);
            if( i < N-1 )
                file << ",";
            file << std::endl;
        }
        file << "  ]," << std::endl;

        file << "  \"theta\": [" << std::endl;
        for(unsigned int i = 0 ; i < N ; i++)
        {
            file << "    " << polar(2,i);
            if( i < N-1 )
                file << ",";
            file << std::endl;
        }
        file << "  ]," << std::endl;

        Eigen::MatrixXd cart = Eigen::MatrixXd::Zero(6,N);
        if( ls.areCartesianDataAvailable())
        {
            cart = ls.getCartesianData();
        }

        file << "  \"xx\": [" << std::endl;
        for(unsigned int i = 0 ; i < N ; i++)
        {
            file << "    " << cart(3,i);
            if( i < N-1 )
                file << ",";
            file << std::endl;
        }
        file << "  ]," << std::endl;

        file << "  \"yy\": [" << std::endl;
        for(unsigned int i = 0 ; i < N ; i++)
        {
            file << "    " << cart(4,i);
            if( i < N-1 )
                file << ",";
            file << std::endl;
        }
        file << "  ]," << std::endl;

        file << "  \"hh\": [" << std::endl;
        for(unsigned int i = 0 ; i < N ; i++)
        {
            file << "    " << cart(5,i);
            if( i < N-1 )
                file << ",";
            file << std::endl;
        }
        file << "  ]," << std::endl;

    }
    file << "  \"size\": " << N << std::endl;
    file << "}" << std::endl;

    file.close();

    return true;
}

