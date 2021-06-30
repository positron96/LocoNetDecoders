#pragma once

#include <Stream.h>

template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; }
template<class T> inline Print &operator <<=(Print &obj, T arg) { obj.println(arg); return obj; }