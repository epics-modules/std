#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/
int addSymbol(const char *name, void *address, epicsType type);
int setSymbol(const char *name, const char *pValue);
int showSymbol(const char *pattern);
#ifdef __cplusplus
}
#endif /*__cplusplus*/


