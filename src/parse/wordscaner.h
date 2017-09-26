
/********************************************************************
* Copyright (c) ，ZJU_ROBOT_TEAM:
* All rights reserved.
* Filename: 	wordscaner.h
* File description：词法扫描模块
* Version：1.0
* Author： Peakulorain
* Date:	19:2:2017
* Modified by：
* Updated date：19:2:2017
*********************************************************************/
#pragma once

#ifndef _WORDSCANER_
#define _WORDSCANER_


#ifdef WIN32
	#include "../RobotApp.h"
	#include"afxwin.h"
	#include "../MainFrm.h"
	#include "../OutputWnd.h"
	#define _WINDOWS_VS_
#endif

#include <math.h>
#include<string>
#include<vector>
#include <queue>
#include"stdlib.h"

//#ifndef WORDSCANER_H_
//#define WORDSCANER_H_
//
//#else
//
//#endif /* WORDSCANER_H_ */

using namespace std;

typedef  enum{
	sASSIGN=1,        // =
	sPLUS,                 // +
	sMINUS,             // -
	sMULT,                // *
	sDIV,                    // /
	sLT,                       // <
	sGT,                      // >
	sLTOE,                 // <=
	sGTOE,                 // >=
    sEQ,                     // ==
	sUEQ,                  // !=
	sSEMIC ,             // ;
	sCOMMA,           // ,
	sLBRAC,              // (
	sRBRAC,              // )
	sLARRSYM,        // [
	sRARRSYM,        // ]
	sLBOUND,         // {
	sRBOUND,         // }
//	sLNOTE,             // /*
//	sRNOTE,             // */
	sBREAK,
	sCONTINUE,

	slIF,                      // IF
	slELSE,                // ELSE
	slWHILE,            // WHILE

	sdINT,                // INT
	sdBOOL,            //BOOL
	sdDOUBLE,       // DOUBLE
	sdSTRING,        //STRING 用于声明字符类型
	sdPOSIT,         //空间点
	sdJOINT,         //关节点
	sdTOOL,          //工具坐标
	sdSPEED,         //速度
	sdDIO,           //dio
	sdSIO,           //sio

	seMOVL,           // MOVL
	seMOVC,           // MOVC
	seMOVJ,           // MOVJ
	seDisaPower,	  //DisablePower
	seEnaPower,       //EnablePower
	sePause,
	seRestart,
	seWaitEnd,
	seDelay,
	ioDIOLINK,    //DioLink
	ioSIOLINK,    //SioLink

	seIsPowered,      //IsPowered
	seIsSettled,
	sNUM,               //数值
	sSTR,          //字符串
	sTRUE,
	sFALSE,

	RFUNC,
	ioDIOGET,     //DioGet
	ioDIOSET,     //DioSet
	ioDIOSTATUS,  //DioStatus
	ioSIOGET,     //SioGet
	ioSIOSET,     //SioSet
	ioCLEARBUFF,  //ClearBuff
	ioSIOCTRL,    //SioCtrl

	BLENDS,       //Blends
	DEFAULT,      //default
	SMOOTHTRA,    //smoothtransit

	sBEGIN,             // BEGIN
	sEND,                 // END

	/***************************************************
	* 函数名:加速度
	* 作者：林旭军
	* 创建日期：2017-1-16
	* 修改日期：2017-1-16
	***************************************************/
	sdACCELERATION //ACCELERATION
	/*********************end**************************/

} M_TokenType;

typedef  enum{
	WAIT=1,
	START,
	DONE
}STATUS;

struct Token
{
	M_TokenType  m_TokenType;
	string tokenString;
	unsigned long int  lineno;         //行号
};

class wordscaner {
public:
	wordscaner();
	virtual ~wordscaner();

public:
	unsigned long int  lineCount;//记录文档行
	string    filestring;//用来储存文件的所有字符
	void      getFileString(string filepath);//用来获得指定路径的全部字符

	string  strCurrent;//用来储存当前字符串
	char chCurrent;//用来储存当前字符
	STATUS status;//当前字符串分析状态
	bool Analyze(string path);//分析
	bool AnalyzeChar(char chCurrent);//分析当前字符
	bool AnalyzeString(string str);//分析当前字符

	bool AnalyzeFragmentString(string str);//分析片段字符串

	void IgnoreOthers(string str) ; //过滤空白和换行符号

    unsigned long int CurrentCharIndex;//用于记录当前字符在整个字符串中的位置
	char getOneChar(string str);//获取一个字符

public:
	void reporterror(int errorflag);

public:
	vector<Token>  TokenList;//用于储存标识符表
	void pushTokenList(string str,M_TokenType tt);

public:///////////////////////////////////////////////////////////编译平台
#ifdef _WINDOWS_VS_
	CRobotAppApp *theApp;
	wordscaner(CRobotAppApp *theApp);
	COutputWnd *p_OutputWnd;
#endif

};


#endif//define _WORDSCANER_
