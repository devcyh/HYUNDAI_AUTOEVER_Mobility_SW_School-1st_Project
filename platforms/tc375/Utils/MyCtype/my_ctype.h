#ifndef UTILS_MYCTYPE_MY_CTYPE_H_
#define UTILS_MYCTYPE_MY_CTYPE_H_

#include <stdbool.h>

static inline bool my_isdigit (char c)
{
    return (c >= '0' && c <= '9');
}

#endif /* UTILS_MYCTYPE_MY_CTYPE_H_ */
