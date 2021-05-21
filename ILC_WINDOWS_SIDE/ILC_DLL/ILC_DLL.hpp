#ifndef ILC_DLL_EXPORTS_HPP
#define ILC_DLL_EXPORTS_HPP

#ifdef ILC_DLL_EXPORTS
#define ILC_DLL_EXPORTS_API extern "C" __declspec(dllexport)
#else
#define ILC_DLL_EXPORTS_API __declspec(dllimport)
#endif

ILC_DLL_EXPORTS_API int CExecuteTrajectory(double* timeStamps, double* ml0, double* mf231, double* ml1, int* n);

#endif // ILC_DLL_EXPORTS_HPP