#pragma once

#define DLLATTRIB

#if defined SHARED_LIBRARY_EXPORT
#undef DLLATTRIB
#define DLLATTRIB __declspec(dllexport)
#elif defined SHARED_LIBRARY_IMPORT
#undef DLLATTRIB
#define DLLATTRIB __declspec(dllimport)
#endif
