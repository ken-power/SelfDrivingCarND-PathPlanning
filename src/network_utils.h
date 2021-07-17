#ifndef PATH_PLANNING_NETWORK_UTILS_H
#define PATH_PLANNING_NETWORK_UTILS_H

#include <cmath>
#include <string>
#include <vector>

using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string GetJsonData(const string& s)
{
    auto found_null = s.find("null");
    auto b1 = s.find_first_of('[');
    auto b2 = s.find_first_of('}');

    if(found_null != string::npos)
    {
        return "";
    }
    else if(b1 != string::npos && b2 != string::npos)
    {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

#endif  // PATH_PLANNING_NETWORK_UTILS_H
