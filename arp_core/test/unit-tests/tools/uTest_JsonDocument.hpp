/*
 * uTest_JsonDocument.hpp
 *
 *  Created on: 2 mars 2012
 *      Author: boris
 */

#include "tools/vjson/JsonDocument.hpp"

using namespace std;
using namespace vjson;

BOOST_AUTO_TEST_SUITE( unittest_JsonDocument )

BOOST_AUTO_TEST_CASE( test_parse )
{
    vjson::JsonDocument doc;
    doc.parse("./scan.json");
}

BOOST_AUTO_TEST_CASE( test_getChildNames )
{
    vjson::JsonDocument doc;
    BOOST_CHECK( doc.parse("./scan.json") );

    std::vector< std::string > vs = doc.getChildNames(doc.root());

    BOOST_CHECK( vs[0].compare("range") == 0 );
    BOOST_CHECK( vs[1].compare("size") == 0 );
    BOOST_CHECK( vs[2].compare("theta") == 0 );
    BOOST_CHECK( vs[3].compare("tt") == 0 );
    BOOST_CHECK( vs[4].compare("type") == 0 );
}

BOOST_AUTO_TEST_CASE( test_getChildTypes )
{
    vjson::JsonDocument doc;
    doc.parse("./scan.json");

    std::vector< json_type > vs = doc.getChildTypes(doc.root());

    BOOST_CHECK( vs[0] == JSON_ARRAY );
    BOOST_CHECK( vs[1] == JSON_INT );
    BOOST_CHECK( vs[2] == JSON_ARRAY );
    BOOST_CHECK( vs[3] == JSON_ARRAY );
    BOOST_CHECK( vs[4] == JSON_STRING );
}

BOOST_AUTO_TEST_CASE( test_getChild_name )
{
    vjson::JsonDocument doc;
    doc.parse("./scan.json");

    {
        json_value * child = doc.getChild(doc.root(), "range");
        BOOST_CHECK( child );
    }

    {
        json_value * child = doc.getChild(doc.root(), "size");
        BOOST_CHECK( child );
    }

    {
        json_value * child = doc.getChild(doc.root(), "theta");
        BOOST_CHECK( child );
    }

    {
        json_value * child = doc.getChild(doc.root(), "tt");
        BOOST_CHECK( child );
    }

    {
        json_value * child = doc.getChild(doc.root(), "type");
        BOOST_CHECK( child );
    }

    {
        json_value * child = doc.getChild(doc.root(), "other");
        BOOST_CHECK( child == NULL );
    }
}

BOOST_AUTO_TEST_CASE( test_getChild_index )
{
    vjson::JsonDocument doc;
    doc.parse("./scan.json");

    {
        json_value * child = doc.getChild(doc.root(), 0);
        BOOST_CHECK( child );
    }

    {
        json_value * child = doc.getChild(doc.root(), 1);
        BOOST_CHECK( child );
    }

    {
        json_value * child = doc.getChild(doc.root(), 2);
        BOOST_CHECK( child );
    }

    {
        json_value * child = doc.getChild(doc.root(), 3);
        BOOST_CHECK( child );
    }

    {
        json_value * child = doc.getChild(doc.root(), 4);
        BOOST_CHECK( child );
    }

    {
        json_value * child = doc.getChild(doc.root(), 5);
        BOOST_CHECK( child == NULL );
    }
}

BOOST_AUTO_TEST_CASE( test_getIntegerData )
{
    vjson::JsonDocument doc;
    doc.parse("./scan.json" );

    json_value * child = doc.getChild(doc.root(), "size");

    int data = doc.getIntegerData(child);
    BOOST_CHECK_EQUAL( data, 681 );
}

BOOST_AUTO_TEST_CASE( test_getStringData )
{
    vjson::JsonDocument doc;
    doc.parse("./scan.json" );

    json_value * child = doc.getChild(doc.root(), "type");

    std::string data = doc.getStringData(child);
    BOOST_CHECK( data.compare("Scan") == 0 );
}

BOOST_AUTO_TEST_CASE( test_getFloatData )
{
    vjson::JsonDocument doc;
    doc.parse("./scan.json" );

    json_value * child = doc.getChild(doc.root(), "tt");

    {
    float data = doc.getFloatData( doc.getChild(child, 0) );
    BOOST_CHECK_CLOSE( data, 0.033496093750000004, 1.f );
    }
    {
    float data = doc.getFloatData( doc.getChild(child, 1) );
    BOOST_CHECK_CLOSE( data, 0.033593749999999999, 1.f  );
    }
    {
    float data = doc.getFloatData( doc.getChild(child, 2) );
    BOOST_CHECK_CLOSE( data, 0.03369140625, 1.f  );
    }

}

BOOST_AUTO_TEST_SUITE_END()
