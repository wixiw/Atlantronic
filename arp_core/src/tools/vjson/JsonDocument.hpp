/*
 * JsonDocument.hpp
 *
 *  Created on: 29 february 2012
 *      Author: Boris
 */

#ifndef _ARP_CORE_TOOLS_JSONDOCUMENT_HPP_
#define _ARP_CORE_TOOLS_JSONDOCUMENT_HPP_

#include <vector>
#include <string>
#include <stdio.h>

#include "tools/vjson/json.h"
#include "tools/vjson/block_allocator.h"

namespace vjson
{

class JsonDocument
{
    private:
        block_allocator mAllocator;
        json_value *mRoot;

    public:
        JsonDocument(): mAllocator(1 << 10), mRoot(0)
        {
        }

        bool parse(const char *filename);

        json_value *root()
        {
            return mRoot;
        }

        std::vector< std::string > getChildNames( json_value* value );
        std::vector< json_type > getChildTypes( json_value* value );
        json_value* getChild( json_value* value, std::string name );
        json_value* getChild( json_value* value, unsigned int index );

        int getIntegerData(json_value * value);
        float getFloatData(json_value * value);
        std::string getStringData(json_value * value);
};


}

#endif //_ARP_CORE_TOOLS_JSONDOCUMENT_HPP_
