/* Bitmask for selecting debug level information */
typedef enum
{
    VPROC_DBG_LVL_NONE = 0x0,
    VPROC_DBG_LVL_FUNC = 0x1,
    VPROC_DBG_LVL_INFO = 0x2,
    VPROC_DBG_LVL_WARN = 0x4,
    VPROC_DBG_LVL_ERR = 0x8,
    VPROC_DBG_LVL_ALL = (VPROC_DBG_LVL_FUNC | VPROC_DBG_LVL_INFO | VPROC_DBG_LVL_WARN | VPROC_DBG_LVL_ERR)
} VPROC_DBG_LVL;

#ifndef DEBUG_LEVEL
#define DEBUG_LEVEL VPROC_DBG_LVL_ERR
#endif

/* Macros defining SSL_print */
#ifdef DEBUG 
extern VPROC_DBG_LVL vproc_dbg_lvl;
#define VPROC_U_DBG_SET_LVL(dbg_lvl) (vproc_dbg_lvl = dbg_lvl)
#define VPROC_U_DBG_PRINT(level,msg,args...)  if(level & vproc_dbg_lvl) {printf("[%s:%d]"msg,__FUNCTION__,__LINE__,##args);} 
#else
#define VPROC_U_DBG_PRINT(level,msg,args...) 
#endif