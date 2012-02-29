/*
 * JsonDocument.cpp
 *
 *  Created on: 29 february 2012
 *      Author: Boris
 *
 */

#include "tools/vjson/JsonDocument.hpp"

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

}
