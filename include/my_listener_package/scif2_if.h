#include "scif2_define.h"

#include <dlfcn.h>

#ifdef __X64
	#define PTR long long
#else
    #define PTR long
#endif

//=================== 通訊執行函式，應該每10~100ms呼叫一次 ======================
typedef int  (*_MainProcess)();
//=================== 函式庫初始化 ===================
typedef int  (*_LibraryInitial)(DLL_USE_SETTING *pUseSetting, int MakerID, char* EncString);

typedef void (*_LibrarySetMemSize) (int MemID, MEM_SIZE *pSize);
typedef int  (*_LibrarySetMemMapping) (int TgrConn, int MemID, MEM_OFFSET *pOffset);        //設定鏡射記憶體對應與偏移
typedef void (*_LibraryDestroy) ();
typedef void (*_LibrarySetDebug) (int level);   //level : 0不顯示   1有錯誤才顯示   2通訊內容全部顯示(除錯用，通訊速度變成每秒一次)
//--==========================取得內部資訊 =================================
typedef int   (*_GetLibraryMsg) (int id );                   //取得一般的資訊
typedef int   (*_GetConnectionMsg) (int TgrConn, int MsgID);         //取得連線通訊的資訊
typedef void  (*_GetConnectionError) (int TgrConn, ERROR_MSG *Msg);  //取得錯誤訊息

//===================區域網路中偵測主機功能=====================================
typedef int  (*_LocalDetectControllers) ();                                    //自動偵測主機功能
typedef int  (*_LocalReadControllerCount) ();                                  //讀取取得的控制器資料筆數
typedef int  (*_LocalReadController) (int Index, LOCAL_CONTROLLER_INFO *Info); //讀取取得的控制器資料
typedef int  (*_RemoteGetMyInfo) (int TgrConn, char *MediaIP);                 //取得本身的網際 IP+Port

//==================================連線功能=================================
typedef int  (*_ConnectSetPwd) (int TgrConn, char *Pwd);     //設定連線密碼
typedef int  (*_ConnectLocalList) (int TgrConn, int Index);  //與取得的控制器清單中的Index值進行連線
typedef int  (*_ConnectLocalIP) (int TgrConn, char* IP);     //直接輸入控制器IP進行連線
typedef int  (*_ConnectRemote) (int TgrConn, char *MediaIP, char* IP, int Port, int Pwd); //連線遠端主機
typedef int  (*_Disconnect) (int TgrConn);                   //中斷連線

//==================================檔案傳輸功能=================================
typedef int  (*_FtpSetConnection) (int TgrConn);                                                     //設定 FTP 索引
//==================================遠端檔案清單
typedef int  (*_FtpGetFileList) (int Folder, char *SubFolder, char *HeadFilter, char *TailFilter);   //取得檔案清單
typedef int  (*_FtpReadFileCount) ();                                                                //讀取FTP檔案清單筆數
typedef int  (*_FtpReadFile) (int Index, FTP_FILE *file);                                            //讀取FTP檔案名稱
//==================================檔案傳輸功能=================================
typedef int  (*_FtpMakeDir) (int Folder, char *DirName);                                             //建立資料夾
typedef int  (*_FtpUpload1File) (int Folder, char *SubFolder, char *Filename, char *LocalFilename);   //上傳檔案
typedef int  (*_FtpDownload1File) (int Folder, char *SubFolder, char *Filename, char *LocalFilename); //下載檔案
typedef int  (*_FtpDelete1File) (int Folder, char *SubFolder, char *Filename);                        //刪除檔案
typedef int  (*_FtpTransferFileReset) ();
typedef int  (*_FtpTransferFileAdd) (int Folder, char *SubFolder, char *Filename, char *LocalFilename);
typedef int  (*_FtpUploadFiles) ();
typedef int  (*_FtpDownloadFiles) ();
typedef int  (*_FtpDeleteFiles) ();
//==================================
typedef int  (*_FtpCheckDone) ();                                             //取得執行結果
typedef int  (*_FtpWaitDone) (int MaxWaitTime);                               //取得執行結果
//==================================本地端檔案清單
typedef int  (*_LocalGetFileList) (char *Path, char *HeadFilter, char *TailFilter);  //取得本地端檔案清單
typedef int  (*_LocalReadFileCount) ();                                              //讀取本地檔案清單筆數
typedef int  (*_LocalReadFile) (int Index, FTP_FILE *file);                          //讀取本地檔案名稱
typedef int  (*_LocalDeleteFile) (int Index);                                        //刪除本地檔案名稱

//====================== 由本地記憶體讀取資料
typedef int  (*_memI) (int TgrConn, int addr );
typedef int  (*_memO) (int TgrConn, int addr );
typedef int  (*_memC) (int TgrConn, int addr );
typedef int  (*_memS) (int TgrConn, int addr );
typedef int  (*_memA) (int TgrConn, int addr );
typedef int  (*_memR) (int TgrConn, int addr );
typedef int  (*_memRBit) (int TgrConn, int addr, int BitIdx);
typedef int  (*_memRString) (int TgrConn, int addr, int RSize, char *Buf );           //由鏡射記憶體中讀取字串
typedef double (*_memF) (int TgrConn, int addr );
typedef char*  (*_memRPtr) (int TgrConn, int addr );
//----
typedef int  (*_memTS) (int TgrConn, int addr );  //20180903
typedef int  (*_memTV) (int TgrConn, int addr );
typedef int  (*_memTT) (int TgrConn, int addr );
typedef int  (*_memCS) (int TgrConn, int addr );
typedef int  (*_memCV) (int TgrConn, int addr );
typedef int  (*_memCT) (int TgrConn, int addr );

//====================== 寫入本地記憶體
typedef int  (*_memSetO) (int TgrConn, int addr, char val);
typedef int  (*_memSetC) (int TgrConn, int addr, char val);
typedef int  (*_memSetS) (int TgrConn, int addr, char val);
typedef int  (*_memSetA) (int TgrConn, int addr, char val);
typedef int  (*_memSetR) (int TgrConn, int addr, int val);
typedef int  (*_memSetRBit) (int TgrConn, int addr, int BitIdx, int val);
typedef int  (*_memSetRString) (int TgrConn, int addr, int RSize, char *Buf );           //由鏡射記憶體中讀取字串
typedef int  (*_memSetF) (int TgrConn, int addr, double val);


//============================= 取得通訊處理狀態
typedef int       (*_GetTranState) (PTR pTran);
typedef SC_DATA*  (*_GetDataPointer)(PTR pTran);
// ========================================================
//以下函式回傳值若為 0 ，代表指令初拒絕，若不為 0 ，代表是該筆通訊的指標
//此時，將其帶入 GetTranState 的引數中，取得該筆通訊的狀態，將會是 SC_TRANSACTION_RESET
//一段時間之後再帶入 GetTranState 的引數中，回傳值若為 SC_TRANSACTION_FINISH 代表該筆通訊已被處理
typedef void (*_LClearQueue) (int TgrConn);             //清除命令
//  ============================= Loop read ============
typedef void (*_LReadBegin) (int TgrConn);        //設定自動組合旗標
typedef PTR  (*_LReadNI) (int TgrConn, int addr, int num);
typedef PTR  (*_LReadNO) (int TgrConn, int addr, int num);
typedef PTR  (*_LReadNC) (int TgrConn, int addr, int num);
typedef PTR  (*_LReadNS) (int TgrConn, int addr, int num);
typedef PTR  (*_LReadNA) (int TgrConn, int addr, int num);
typedef PTR  (*_LReadNR) (int TgrConn, int addr, int num );
typedef PTR  (*_LReadNF) (int TgrConn, int addr, int num );
typedef PTR  (*_LReadNTS) (int TgrConn, int addr, int num );    //20180903
typedef PTR  (*_LReadNTV) (int TgrConn, int addr, int num );
typedef PTR  (*_LReadNTT) (int TgrConn, int addr, int num );
typedef PTR  (*_LReadNCS) (int TgrConn, int addr, int num );
typedef PTR  (*_LReadNCV) (int TgrConn, int addr, int num );
typedef PTR  (*_LReadNCT) (int TgrConn, int addr, int num );
typedef PTR  (*_LReadEnd)(int TgrConn);        //完成自動組合設定並開始產生組合封包
//-----

//  ============================= L write ============
typedef void (*_LWriteBegin) (int TgrConn);        //設定自動組合旗標
typedef PTR  (*_LWriteNO) (int TgrConn,  int addr, int num);
typedef PTR  (*_LWriteNC) (int TgrConn,  int addr, int num);
typedef PTR  (*_LWriteNS) (int TgrConn,  int addr, int num);
typedef PTR  (*_LWriteNA) (int TgrConn,  int addr, int num);
typedef PTR  (*_LWriteNR) (int TgrConn,  int addr, int num);
typedef PTR  (*_LWriteNF) (int TgrConn,  int addr, int num);
typedef PTR  (*_LWriteEnd)(int TgrConn);        //完成自動組合設定並開始產生組合封包


typedef void (*_DClearQueue) (int TgrConn);             //清除命令
typedef int  (*_DWaitDone) (int TgrConn, int MaxWaitTime);  //等待直接命令完成
//  ============================= Direct read ============
typedef void (*_DReadBegin) (int TgrConn);       //設定自動組合旗標
typedef PTR  (*_DReadNI) (int TgrConn, int addr, int num);
typedef PTR  (*_DReadNO) (int TgrConn, int addr, int num);
typedef PTR  (*_DReadNC) (int TgrConn, int addr, int num);
typedef PTR  (*_DReadNS) (int TgrConn, int addr, int num);
typedef PTR  (*_DReadNA) (int TgrConn, int addr, int num);
typedef PTR  (*_DReadNR) (int TgrConn, int addr, int num );
typedef PTR  (*_DReadNF) (int TgrConn, int addr, int num );
typedef PTR  (*_DReadNTS) (int TgrConn, int addr, int num );    //20180903
typedef PTR  (*_DReadNTV) (int TgrConn, int addr, int num );
typedef PTR  (*_DReadNTT) (int TgrConn, int addr, int num );
typedef PTR  (*_DReadNCS) (int TgrConn, int addr, int num );
typedef PTR  (*_DReadNCV) (int TgrConn, int addr, int num );
typedef PTR  (*_DReadNCT) (int TgrConn, int addr, int num );
typedef PTR  (*_DReadEnd) (int TgrConn);      //完成自動組合設定並開始產生組合封包


//  ============================= Direct write ============
typedef void (*_DWriteBegin) (int TgrConn);       //設定自動組合旗標
typedef PTR  (*_DWrite1O) (int TgrConn, int addr, int val);
typedef PTR  (*_DWrite1C) (int TgrConn, int addr, int val);
typedef PTR  (*_DWrite1S) (int TgrConn, int addr, int val);
typedef PTR  (*_DWrite1A) (int TgrConn, int addr, int val);
typedef PTR  (*_DWrite1R) (int TgrConn,  int addr, int val);
typedef PTR  (*_DWrite1F) (int TgrConn,  int addr, double val);
typedef PTR  (*_DWrite1RBit) (int TgrConn, int addr, int BitIdx, int BitValue);             //寫入 R bit
typedef PTR  (*_DWriteRString) (int TgrConn, int addr, int RSize, char *Buf);            //寫入字串到 R
typedef PTR  (*_DWriteNO) (int TgrConn,  int addr, int num, int *data);
typedef PTR  (*_DWriteNC) (int TgrConn,  int addr, int num, int *data);
typedef PTR  (*_DWriteNS) (int TgrConn,  int addr, int num, int *data);
typedef PTR  (*_DWriteNA) (int TgrConn,  int addr, int num, int *data);
typedef PTR  (*_DWriteNR) (int TgrConn,  int addr, int num, int *data);
typedef PTR  (*_DWriteNF) (int TgrConn,  int addr, int num, double *data);
typedef PTR  (*_DWriteEnd) (int TgrConn);      //完成自動組合設定並開始產生組合封包


struct SC2
{
    _MainProcess          MainProcess;
    _LibraryInitial       LibraryInitial;
    _LibrarySetMemSize    LibrarySetMemSize;
    _LibrarySetMemMapping LibrarySetMemMapping;
    _LibraryDestroy       LibraryDestroy;
    _LibrarySetDebug      LibrarySetDebug;
    _GetLibraryMsg        GetLibraryMsg;
    _GetConnectionMsg     GetConnectionMsg;
    _GetConnectionError   GetConnectionError;
    _LocalDetectControllers   LocalDetectControllers;
    _LocalReadControllerCount LocalReadControllerCount;
    _LocalReadController      LocalReadController;
    _RemoteGetMyInfo          RemoteGetMyInfo;

    _ConnectSetPwd    ConnectSetPwd;
    _ConnectLocalList ConnectLocalList;
    _ConnectLocalIP   ConnectLocalIP;
    _ConnectRemote    ConnectRemote;
    _Disconnect       Disconnect;

    _FtpSetConnection FtpSetConnection;
    _FtpGetFileList   FtpGetFileList;
    _FtpReadFileCount FtpReadFileCount;
    _FtpReadFile      FtpReadFile;

    _FtpMakeDir           FtpMakeDir;
    _FtpUpload1File       FtpUpload1File;
    _FtpDownload1File     FtpDownload1File;
    _FtpDelete1File       FtpDelete1File;
    _FtpTransferFileReset FtpTransferFileReset;
    _FtpTransferFileAdd   FtpTransferFileAdd;
    _FtpUploadFiles       FtpUploadFiles;
    _FtpDownloadFiles     FtpDownloadFiles;
    _FtpDeleteFiles       FtpDeleteFiles;
    _FtpCheckDone         FtpCheckDone;
    _FtpWaitDone          FtpWaitDone;
    _LocalGetFileList     LocalGetFileList;
    _LocalReadFileCount   LocalReadFileCount;
    _LocalReadFile        LocalReadFile;
    _LocalDeleteFile      LocalDeleteFile;

    _memI memI;
    _memO memO;
    _memC memC;
    _memS memS;
    _memA memA;
    _memR memR;
    _memRBit memRBit;
    _memRString memRString;
    _memF memF;
    _memRPtr memRPtr;
    _memTS memTS;
    _memTV memTV;
    _memTT memTT;
    _memCS memCS;
    _memCV memCV;
    _memCT memCT;

    _memSetO memSetO;
    _memSetC memSetC;
    _memSetS memSetS;
    _memSetA memSetA;
    _memSetR memSetR;
    _memSetRBit memSetRBit;
    _memSetRString memSetRString;
    _memSetF memSetF;

    _GetTranState GetTranState;
    _GetDataPointer GetDataPointer;

    _LClearQueue LClearQueue;
    _LReadBegin LReadBegin;
    _LReadNI LReadNI;
    _LReadNO LReadNO;
    _LReadNC LReadNC;
    _LReadNS LReadNS;
    _LReadNA LReadNA;
    _LReadNR LReadNR;
    _LReadNF LReadNF;
    _LReadNTS LReadNTS;
    _LReadNTV LReadNTV;
    _LReadNTT LReadNTT;
    _LReadNCS LReadNCS;
    _LReadNCV LReadNCV;
    _LReadNCT LReadNCT;
    _LReadEnd LReadEnd;

    _LWriteBegin LWriteBegin;
    _LWriteNO LWriteNO;
    _LWriteNC LWriteNC;
    _LWriteNS LWriteNS;
    _LWriteNA LWriteNA;
    _LWriteNR LWriteNR;
    _LWriteNF LWriteNF;
    _LWriteEnd LWriteEnd;

    _DClearQueue DClearQueue;
    _DWaitDone DWaitDone;

    _DReadBegin DReadBegin;
    _DReadNI DReadNI;
    _DReadNO DReadNO;
    _DReadNC DReadNC;
    _DReadNS DReadNS;
    _DReadNA DReadNA;
    _DReadNR DReadNR;
    _DReadNF DReadNF;
    _DReadNTS DReadNTS;
    _DReadNTV DReadNTV;
    _DReadNTT DReadNTT;
    _DReadNCS DReadNCS;
    _DReadNCV DReadNCV;
    _DReadNCT DReadNCT;
    _DReadEnd DReadEnd;

    _DWriteBegin DWriteBegin;
    _DWrite1O DWrite1O;
    _DWrite1C DWrite1C;
    _DWrite1S DWrite1S;
    _DWrite1A DWrite1A;
    _DWrite1R DWrite1R;
    _DWrite1F DWrite1F;
    _DWrite1RBit DWrite1RBit;
    _DWriteRString DWriteRString;
    _DWriteNO DWriteNO;
    _DWriteNC DWriteNC;
    _DWriteNS DWriteNS;
    _DWriteNA DWriteNA;
    _DWriteNR DWriteNR;
    _DWriteNF DWriteNF;
    _DWriteEnd DWriteEnd;
};



inline int LoadFunction(char *path, SC2 *sc)
{

    //void *lh = dlopen("/home/lnc/scif2_linuxArmSo/exe_LinuxArmSo/libSCIF2_arm.so", RTLD_LAZY);
    void *lh = dlopen(path, RTLD_LAZY);
    if (!lh){
        fprintf(stderr, "dlopen error: %s\n", dlerror());
        return 1;
    }
    printf("libSCIF2_arm.so is loaded\n");

    sc->LibraryInitial            = (_LibraryInitial)dlsym(lh, "LibraryInitial");
    sc->MainProcess               = (_MainProcess)dlsym(lh, "MainProcess");

    sc->LibrarySetMemSize         = (_LibrarySetMemSize)dlsym(lh, "LibrarySetMemSize");
    sc->LibrarySetMemMapping      = (_LibrarySetMemMapping)dlsym(lh, "LibrarySetMemMapping");
    sc->LibraryDestroy            = (_LibraryDestroy)dlsym(lh, "LibraryDestroy");
    sc->LibrarySetDebug           = (_LibrarySetDebug)dlsym(lh, "LibrarySetDebug");
    sc->GetLibraryMsg             = (_GetLibraryMsg)dlsym(lh, "GetLibraryMsg");
    sc->GetConnectionMsg          = (_GetConnectionMsg)dlsym(lh, "GetConnectionMsg");
    sc->GetConnectionError        = (_GetConnectionError)dlsym(lh, "GetConnectionError");
    sc->LocalDetectControllers    = (_LocalDetectControllers)dlsym(lh, "LocalDetectControllers");
    sc->LocalReadControllerCount  = (_LocalReadControllerCount)dlsym(lh, "LocalReadControllerCount");
    sc->LocalReadController       = (_LocalReadController)dlsym(lh, "LocalReadController");
    sc->RemoteGetMyInfo           = (_RemoteGetMyInfo)dlsym(lh, "RemoteGetMyInfo");

    sc->ConnectSetPwd           = (_ConnectSetPwd)dlsym(lh, "ConnectSetPwd");
    sc->ConnectLocalList        = (_ConnectLocalList)dlsym(lh, "ConnectLocalList");
    sc->ConnectLocalIP          = (_ConnectLocalIP)dlsym(lh, "ConnectLocalIP");
    sc->ConnectRemote           = (_ConnectRemote)dlsym(lh, "ConnectRemote");
    sc->Disconnect              = (_Disconnect)dlsym(lh, "Disconnect");

    sc->FtpSetConnection        = (_FtpSetConnection)dlsym(lh, "FtpSetConnection");
    sc->FtpGetFileList          = (_FtpGetFileList)dlsym(lh, "FtpGetFileList");
    sc->FtpReadFileCount        = (_FtpReadFileCount)dlsym(lh, "FtpReadFileCount");
    sc->FtpReadFile             = (_FtpReadFile)dlsym(lh, "FtpReadFile");

    sc->FtpMakeDir             = (_FtpMakeDir)dlsym(lh, "FtpMakeDir");
    sc->FtpUpload1File         = (_FtpUpload1File)dlsym(lh, "FtpUpload1File");
    sc->FtpDownload1File       = (_FtpDownload1File)dlsym(lh, "FtpDownload1File");
    sc->FtpDelete1File         = (_FtpDelete1File)dlsym(lh, "FtpDelete1File");
    sc->FtpTransferFileReset   = (_FtpTransferFileReset)dlsym(lh, "FtpTransferFileReset");
    sc->FtpTransferFileAdd     = (_FtpTransferFileAdd)dlsym(lh, "FtpTransferFileAdd");
    sc->FtpUploadFiles         = (_FtpUploadFiles)dlsym(lh, "FtpUploadFiles");
    sc->FtpDownloadFiles       = (_FtpDownloadFiles)dlsym(lh, "FtpDownloadFiles");
    sc->FtpDeleteFiles         = (_FtpDeleteFiles)dlsym(lh, "FtpDeleteFiles");
    sc->FtpCheckDone           = (_FtpCheckDone)dlsym(lh, "FtpCheckDone");
    sc->FtpWaitDone            = (_FtpWaitDone)dlsym(lh, "FtpWaitDone");
    sc->LocalGetFileList       = (_LocalGetFileList)dlsym(lh, "LocalGetFileList");
    sc->LocalReadFileCount     = (_LocalReadFileCount)dlsym(lh, "LocalReadFileCount");
    sc->LocalReadFile          = (_LocalReadFile)dlsym(lh, "LocalReadFile");
    sc->LocalDeleteFile        = (_LocalDeleteFile)dlsym(lh, "LocalDeleteFile");

    sc->memI             = (_memI)dlsym(lh, "memI");
    sc->memO             = (_memO)dlsym(lh, "memO");
    sc->memC             = (_memC)dlsym(lh, "memC");
    sc->memS             = (_memS)dlsym(lh, "memS");
    sc->memS             = (_memS)dlsym(lh, "memS");
    sc->memA             = (_memA)dlsym(lh, "memA");
    sc->memR             = (_memR)dlsym(lh, "memR");
    sc->memRBit          = (_memRBit)dlsym(lh, "memRBit");
    sc->memRString       = (_memRString)dlsym(lh, "memRString");
    sc->memF             = (_memF)dlsym(lh, "memF");
    sc->memRPtr          = (_memRPtr)dlsym(lh, "memRPtr");

    sc->memTS             = (_memTS)dlsym(lh, "memTS");
    sc->memTV             = (_memTV)dlsym(lh, "memTV");
    sc->memTT             = (_memTT)dlsym(lh, "memTT");
    sc->memCS             = (_memCS)dlsym(lh, "memCS");
    sc->memCV             = (_memCV)dlsym(lh, "memCV");
    sc->memCT             = (_memCT)dlsym(lh, "memCT");

    sc->memSetO             = (_memSetO)dlsym(lh, "memSetO");
    sc->memSetC             = (_memSetC)dlsym(lh, "memSetC");
    sc->memSetS             = (_memSetS)dlsym(lh, "memSetS");
    sc->memSetA             = (_memSetA)dlsym(lh, "memSetA");
    sc->memSetR             = (_memSetR)dlsym(lh, "memSetR");
    sc->memSetRBit          = (_memSetRBit)dlsym(lh, "memSetRBit");
    sc->memSetRString       = (_memSetRString)dlsym(lh, "memSetRString");
    sc->memSetF             = (_memSetF)dlsym(lh, "memSetF");

    sc->GetTranState        = (_GetTranState)dlsym(lh, "GetTranState");
    sc->GetDataPointer      = (_GetDataPointer)dlsym(lh, "GetDataPointer");

    sc->LClearQueue         = (_LClearQueue)dlsym(lh, "LClearQueue");
    sc->LReadBegin          = (_LReadBegin)dlsym(lh, "LReadBegin");
    sc->LReadNI             = (_LReadNI)dlsym(lh, "LReadNI");
    sc->LReadNO             = (_LReadNO)dlsym(lh, "LReadNO");
    sc->LReadNC             = (_LReadNC)dlsym(lh, "LReadNC");
    sc->LReadNS             = (_LReadNS)dlsym(lh, "LReadNS");
    sc->LReadNA             = (_LReadNA)dlsym(lh, "LReadNA");
    sc->LReadNR             = (_LReadNR)dlsym(lh, "LReadNR");

    sc->LReadNF              = (_LReadNF)dlsym(lh, "LReadNF");
    sc->LReadNTS             = (_LReadNTS)dlsym(lh, "LReadNTS");
    sc->LReadNTV             = (_LReadNTV)dlsym(lh, "LReadNTV");
    sc->LReadNTT             = (_LReadNTT)dlsym(lh, "LReadNTT");
    sc->LReadNCS             = (_LReadNCS)dlsym(lh, "LReadNCS");
    sc->LReadNCV             = (_LReadNCV)dlsym(lh, "LReadNCV");
    sc->LReadNCT             = (_LReadNCT)dlsym(lh, "LReadNCT");
    sc->LReadEnd             = (_LReadEnd)dlsym(lh, "LReadEnd");

    sc->LWriteBegin          = (_LWriteBegin)dlsym(lh, "LWriteBegin");
    sc->LWriteNO             = (_LWriteNO)dlsym(lh, "LWriteNO");
    sc->LWriteNC             = (_LWriteNC)dlsym(lh, "LWriteNC");
    sc->LWriteNS             = (_LWriteNS)dlsym(lh, "LWriteNS");
    sc->LWriteNA             = (_LWriteNA)dlsym(lh, "LWriteNA");
    sc->LWriteNR             = (_LWriteNR)dlsym(lh, "LWriteNR");
    sc->LWriteNF             = (_LWriteNF)dlsym(lh, "LWriteNF");
    sc->LWriteEnd            = (_LWriteEnd)dlsym(lh, "LWriteEnd");

    sc->DClearQueue          = (_DClearQueue)dlsym(lh, "DClearQueue");
    sc->DWaitDone            = (_DWaitDone)dlsym(lh, "DWaitDone");

    sc->DReadBegin          = (_DReadBegin)dlsym(lh, "DReadBegin");
    sc->DReadNI             = (_DReadNI)dlsym(lh, "DReadNI");
    sc->DReadNO             = (_DReadNO)dlsym(lh, "DReadNO");
    sc->DReadNC             = (_DReadNC)dlsym(lh, "DReadNC");
    sc->DReadNS             = (_DReadNS)dlsym(lh, "DReadNS");
    sc->DReadNA             = (_DReadNA)dlsym(lh, "DReadNA");
    sc->DReadNR             = (_DReadNR)dlsym(lh, "DReadNR");
    sc->DReadNF             = (_DReadNF)dlsym(lh, "DReadNF");
    sc->DReadNTS             = (_DReadNTS)dlsym(lh, "DReadNTS");
    sc->DReadNTV             = (_DReadNTV)dlsym(lh, "DReadNTV");
    sc->DReadNTT             = (_DReadNTT)dlsym(lh, "DReadNTT");
    sc->DReadNCS             = (_DReadNCS)dlsym(lh, "DReadNCS");
    sc->DReadNCV             = (_DReadNCV)dlsym(lh, "DReadNCV");
    sc->DReadNCT             = (_DReadNCT)dlsym(lh, "DReadNCT");
    sc->DReadEnd             = (_DReadEnd)dlsym(lh, "DReadEnd");

    sc->DWriteBegin          = (_DWriteBegin)dlsym(lh, "DWriteBegin");
    sc->DWrite1O             = (_DWrite1O)dlsym(lh, "DWrite1O");
    sc->DWrite1C             = (_DWrite1C)dlsym(lh, "DWrite1C");
    sc->DWrite1S             = (_DWrite1S)dlsym(lh, "DWrite1S");
    sc->DWrite1A             = (_DWrite1A)dlsym(lh, "DWrite1A");
    sc->DWrite1R             = (_DWrite1R)dlsym(lh, "DWrite1R");
    sc->DWrite1F             = (_DWrite1F)dlsym(lh, "DWrite1F");
    sc->DWrite1RBit          = (_DWrite1RBit)dlsym(lh, "DWrite1RBit");
    sc->DWriteRString        = (_DWriteRString)dlsym(lh, "DWriteRString");

    sc->DWriteNO             = (_DWriteNO)dlsym(lh, "DWriteNO");
    sc->DWriteNC             = (_DWriteNC)dlsym(lh, "DWriteNC");
    sc->DWriteNS             = (_DWriteNS)dlsym(lh, "DWriteNS");
    sc->DWriteNA             = (_DWriteNA)dlsym(lh, "DWriteNA");
    sc->DWriteNR             = (_DWriteNR)dlsym(lh, "DWriteNR");
    sc->DWriteNF             = (_DWriteNF)dlsym(lh, "DWriteNF");
    sc->DWriteEnd            = (_DWriteEnd)dlsym(lh, "DWriteEnd");
    
}



