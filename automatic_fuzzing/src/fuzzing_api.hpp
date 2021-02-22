#include <cstdint>
#include <cstdlib>
#include <string>

/* Fuzzing API */
bool getBool(bool& b);
bool getByte(uint8_t& b);
bool getChar(char& c);

bool getFloat32(float& f);
bool getFloat64(double& d);

bool getInt8(int8_t& i);
bool getUInt8(uint8_t& ui);

bool getInt16(int16_t& i);
bool getUInt16(uint16_t& ui);

bool getInt32(int32_t& i);
bool getUInt32(uint32_t& ui);

bool getInt64(int64_t& i);
bool getUInt64(uint64_t& ui);

bool getString(std::string& s, uint8_t size);

/* Implementation */
bool getBool(bool& b)
{
    int c = getchar();
    if (c == EOF) return false;
    b = (c % 2 == 0);
    return true;
}

bool getByte(uint8_t& b)
{
    int c = getchar();
    if (c == EOF) return false;
    b = c;
    return true;
}

bool getChar(char& c)
{
    int gc = getchar();
    if (gc == EOF) return false;
    c = gc;
    return true;
}

bool getFloat32(float& f)
{
    char* bytes = (char *)& f;
    for (size_t i = 0; i < sizeof (float); ++i) {
        int c = getchar ();
        if (c == EOF) return false;
        bytes[i] = (char) c;
    }
    return true;
}

bool getFloat64(double& d)
{
    char* bytes = (char *)& d;
    for (size_t i = 0; i < sizeof (double); ++i) {
        int c = getchar ();
        if (c == EOF) return false;
        bytes[i] = (char) c;
    }
    return true;
}

bool getInt8(int8_t& i)
{
    int c = getchar();
    i = c;
    return (c != EOF);
}

bool getUInt8(uint8_t& ui)
{
    return getInt8((int8_t& )ui);
}

bool getInt16(int16_t& i) {
    char* bytes = (char *)& i;
    for (size_t i = 0; i < sizeof(int16_t); ++i)
    {
        int c = getchar();
        if (c == EOF) return false;
        bytes[i] = (char) c;
    }
    return true;
}

bool getUInt16(uint16_t& ui) {
    return getInt16((int16_t&) ui);
}

bool getInt32(int32_t& i) {
    char* bytes = (char *)& i;
    for (size_t i = 0; i < sizeof(int32_t); ++i)
    {
        int c = getchar();
        if (c == EOF) return false;
        bytes[i] = (char) c;
    }
    return true;
}

bool getUInt32(uint32_t& ui) {
    return getInt32((int32_t&) ui);
}

bool getInt64(int64_t& i)
{
    char* bytes = (char *)& i;
    for (size_t i = 0; i < sizeof(int64_t); ++i)
    {
        int c = getchar();
        if (c == EOF) return false;
        bytes[i] = (char) c;
    }
    return true;
}

bool getUInt64(uint64_t& ui)
{
    return getInt64((int64_t&) ui);
}

bool getString(std::string& s, uint8_t size)
{
    s = "";
    for (size_t i = 0; i < size; ++i)
    {
        int c = getchar();
        if (c == EOF) return false;
        s += (char)c;
    }
    return true;
}