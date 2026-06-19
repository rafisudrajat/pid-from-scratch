#include <Python.h>
#include "matplotlibcpp.h"
#include <string>
#include <cstdlib>

// Convert a narrow string to wide — the static storage keeps the
// pointer valid for the lifetime of the program, which is what
// Py_SetPythonHome requires.
static const wchar_t* toWide(const char* s) {
    static std::wstring ws;
    ws.assign(s, s + std::char_traits<char>::length(s));
    return ws.c_str();
}

// Initialize the embedded Python interpreter.
//
// The interpreter needs two things that aren't obvious at runtime:
//
//   1. PYTHONHOME — the base Python prefix where the standard library
//      lives (encodings/, os.py, etc.).  For a venv, this is the
//      installation the venv was created from, not the venv directory
//      itself.  Discovered at CMake configure time via
//      `sys.base_prefix` and baked in as PYTHON3_BASE_PREFIX.
//
//   2. Site-packages — where the .venv's pip-installed packages live
//      (matplotlib, numpy).  Discovered at CMake configure time via
//      `sysconfig.get_path('purelib')` and baked in as
//      PYTHON3_SITE_PACKAGES.  We prepend it to the PYTHONPATH
//      environment variable before Py_Initialize().
//
// Both paths come from the project-local .venv, so no hardcoded
// machine-specific paths are needed.  The same logic works on Linux
// and Windows — only the path separator differs (: vs ;).
void matplotlibcpp::initialize_python(){
    // Tell the embedded interpreter where the standard library lives
    Py_SetPythonHome(toWide(PYTHON3_BASE_PREFIX));

    // Prepend the .venv's site-packages to PYTHONPATH so matplotlib
    // and numpy are importable.  Preserve any existing PYTHONPATH.
#ifdef _WIN32
    const char pathSep = ';';
    const char* existing = getenv("PYTHONPATH");
    if (existing && existing[0] != '\0') {
        std::string combined = std::string(PYTHON3_SITE_PACKAGES) + pathSep + existing;
        _putenv_s("PYTHONPATH", combined.c_str());
    } else {
        _putenv_s("PYTHONPATH", PYTHON3_SITE_PACKAGES);
    }
#else
    const char pathSep = ':';
    const char* existing = getenv("PYTHONPATH");
    if (existing && existing[0] != '\0') {
        std::string combined = std::string(PYTHON3_SITE_PACKAGES) + pathSep + existing;
        setenv("PYTHONPATH", combined.c_str(), 1);
    } else {
        setenv("PYTHONPATH", PYTHON3_SITE_PACKAGES, 1);
    }
#endif

    Py_Initialize();
}
