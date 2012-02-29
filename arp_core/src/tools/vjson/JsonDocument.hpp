/*
 * JsonDocument.hpp
 *
 *  Created on: 29 february 2012
 *      Author: Boris
 */

#ifndef _ARP_CORE_TOOLS_JSONDOCUMENT_HPP_
#define _ARP_CORE_TOOLS_JSONDOCUMENT_HPP_

#include <vector>
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
};

}

#endif //_ARP_CORE_TOOLS_JSONDOCUMENT_HPP_
