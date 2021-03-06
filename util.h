#pragma once

#include <stdlib.h>
#include <cmath>

inline bool parseFloatStrict( const char *str, int len, float& value ) {
    char *parseEnd;
    value = strtod(str, &parseEnd);

    const char *expectEnd = str + len;
    if( expectEnd != parseEnd ) return false;
    return true;
}
