/*
 * JsonDocument.cpp
 *
 *  Created on: 29 february 2012
 *      Author: Boris
 *
 */

#include "tools/vjson/JsonDocument.hpp"

#include <iostream>

namespace vjson
{

bool JsonDocument::parse(const char *filename)
{
    FILE *fp = fopen(filename, "rb");
    if (!fp)
    {
        printf("Can't open %s\n", filename);
        return false;
    }

    fseek(fp, 0, SEEK_END);
    int size = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    char *source = (char *)mAllocator.malloc(size + 1);
    fread(source, 1, size, fp);
    source[size] = 0;

    fclose(fp);

    char *errorPos = 0;
    char *errorDesc = 0;
    int errorLine = 0;
    mRoot = json_parse(source, &errorPos, &errorDesc, &errorLine, &mAllocator);
    if (!mRoot)
    {
        printf("Error at line %d: %s\n%s\n\n", errorLine, errorDesc, errorPos);
        return false;
    }

    return true;
}

std::vector< std::string > JsonDocument::getChildNames( json_value* value )
{
    std::vector< std::string > out;
    if(!value)
        return out;
    for (json_value *it = value->first_child; it; it = it->next_sibling)
    {
        std::string childName;
        if(it->name)
            childName = std::string(it->name);
        out.push_back(childName);
    }
    return out;
}

std::vector< json_type > JsonDocument::getChildTypes( json_value* value )
{
    std::vector< json_type > out;
    if(!value)
        return out;
    for (json_value *it = value->first_child; it; it = it->next_sibling)
    {
        out.push_back(it->type);
    }
    return out;
}

json_value* JsonDocument::getChild( json_value* value, std::string childName )
{
    if(!value)
        return NULL;
    for (json_value *it = value->first_child; it; it = it->next_sibling)
    {
        std::string itName;
        if(it->name)
            itName = std::string(it->name);
        if( childName.compare(std::string(itName)) == 0 )
            return it;
    }
    return NULL;
}

json_value* JsonDocument::getChild( json_value* value, unsigned int index )
{
    if(!value)
        return NULL;
    unsigned int i = 0;
    for (json_value *it = value->first_child; it; it = it->next_sibling, i++)
    {
        if(i == index)
            return it;
    }
    return NULL;
}

int JsonDocument::getIntegerData(json_value * value)
{
    if(!value)
        return 0;
    if(value->type == JSON_INT)
    {
        return value->int_value;
    }
    return 0;
}

float JsonDocument::getFloatData(json_value * value)
{
    if(!value)
        return 0.;
    if(value->type == JSON_FLOAT)
    {
        return value->float_value;
    }
    return 0.;
}

std::string JsonDocument::getStringData(json_value * value)
{
    if(!value)
        return std::string();
    if(value->type == JSON_STRING)
    {
        return value->string_value;
    }
    return std::string();
}


}
