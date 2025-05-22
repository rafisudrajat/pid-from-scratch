#include <Python.h>
#include "matplotlibcpp.h"


// Define the initialize_python function declared in matplotlibcpp.h
void matplotlibcpp::initialize_python(){
    Py_SetPythonHome(L"C:\\Users\\Rafi\\miniconda3\\envs\\MLEnv");
    Py_SetPath(
        L"C:\\Users\\Rafi\\miniconda3\\envs\\MLEnv\\Lib;"
        L"C:\\Users\\Rafi\\miniconda3\\envs\\MLEnv\\DLLs;"
        L"C:\\Users\\Rafi\\miniconda3\\envs\\MLEnv\\Lib\\site-packages"
    );
    Py_Initialize();
}



