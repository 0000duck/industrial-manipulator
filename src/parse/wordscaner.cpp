#include "wordscaner.h"
#include<iostream>
#include<fstream>
#include<stdlib.h>

using namespace std;

wordscaner::~wordscaner() {
	// TODO Auto-generated destructor stub
	TokenList.clear();
}
//----------------------------------------------------------------------
// Name:		getFileString
// Function:	获得文档所有字符串
// Argument:
// Return:		void
// Author:		Peakulorain
// Date:		19:2:2017
// Modifier:
// Date:		2017-02-19
//----------------------------------------------------------------------
void wordscaner::getFileString(string filepath)
{
	ifstream infile(filepath.c_str());
	string temp;
	if(!infile)
	{
		reporterror(0);
		return;
	}
	while(getline(infile,temp))
	{
		filestring+=temp;
		filestring+="\n";
	}
	infile.close();
}

char wordscaner::getOneChar(string str)
{
	char ch;
	if(CurrentCharIndex<str.length())
	{
		ch=str[CurrentCharIndex];
		CurrentCharIndex++;
	}else{
		ch='\0';
		status=DONE;//分析完成
	}
	return ch;
}

bool wordscaner::Analyze(string path)
{
	status=START;//分析状态开启
	getFileString(path);//获取文档字符储存到filestring中，统计字符串中的行数
	if (filestring=="")
	{
		reporterror(3);
		return false;
	}
	while(status==START)
	{
		chCurrent=getOneChar(filestring);
		if(!AnalyzeChar(chCurrent))
			return false;
	}
	if(status==DONE){//一个文档分析完成扫描器全部初始化
#ifdef _WINDOWS_VS_
	 wordscaner(this->theApp);
	 p_OutputWnd->ShowMessageInDeubugWnd("词法无错误，词法分析完成！");
	 return true;
#else
	 wordscaner();
	 return true;
#endif
	}
	return false;
}

bool wordscaner::AnalyzeString(string str)
{
	if(str=="IF")
		pushTokenList(str,slIF);
	else if(str=="ELSE")
		pushTokenList(str,slELSE);
	else if(str== "WHILE")
		pushTokenList(str,slWHILE);
	else if(str== "INT")
		pushTokenList(str,sdINT);
	else if(str=="BOOL")
		pushTokenList(str,sdBOOL);
	else if(str=="STRING")
		pushTokenList(str,sdSTRING);
	else if(str=="DOUBLE")
		pushTokenList(str,sdDOUBLE);
	else if(str=="POSIT")
		pushTokenList(str,sdPOSIT);
	else if(str=="TOOL")
		pushTokenList(str,sdTOOL);
	else if(str=="SPEED")
		pushTokenList(str,sdSPEED);
	else if(str== "MOVL")
		pushTokenList(str,seMOVL);
	else if(str=="MOVC")
		pushTokenList(str,seMOVC);
	else if(str=="MOVJ")
		pushTokenList(str,seMOVJ);
	else if(str== "BEGIN")
		pushTokenList(str,sBEGIN);
	else if(str== "END")
		pushTokenList(str,sEND);
	else if(str== "TRUE")
		pushTokenList(str,sTRUE);
	else if(str== "FALSE")
		pushTokenList(str,sFALSE);
	else if(str== "DisablePower")
		pushTokenList(str,seDisaPower);
	else if(str== "EnablePower")
		pushTokenList(str,seEnaPower);
	else if(str== "JOINT")
		pushTokenList(str,sdJOINT);
	else if(str== "IsPowered")
		pushTokenList(str,seIsPowered);
	else if(str== "BREAK")
		pushTokenList(str,sBREAK);
	else if(str== "CONTINUE")
		pushTokenList(str,sCONTINUE);
	else if(str== "DIO")
		pushTokenList(str,sdDIO);
	else if(str== "SIO")
		pushTokenList(str,sdSIO);
	else if(str== "Pause")
		pushTokenList(str,sePause);
	else if(str== "Restart")
		pushTokenList(str,seRestart);
	else if(str== "Delay")
		pushTokenList(str,seDelay);
	else if(str== "WaitEnd")
		pushTokenList(str,seWaitEnd);
	else if(str== "IsSettled")
		pushTokenList(str,seIsSettled);
	else if(str== "BLENDS")
		pushTokenList(str,BLENDS);
	else if(str== "DEFAULT")
		pushTokenList(str,DEFAULT);
	else if(str== "SMOOTHTRA")
		pushTokenList(str,SMOOTHTRA);
	//ioDIOLINK,    //DioLink
	//ioDIOGET,     //DioGet
	//ioDIOSET,     //DioSet
	//ioDIOSTATUS,  //DioStatus
	else if(str== "DioLink")
		pushTokenList(str,ioDIOLINK);
	else if(str== "DioGet")
		pushTokenList(str,ioDIOGET);
	else if(str== "DioSet")
		pushTokenList(str,ioDIOSET);
	else if(str== "DioStatus")
		pushTokenList(str,ioDIOSTATUS);
	//ioSIOLINK,    //SioLink
	//ioSIOGET,     //SioGet
	//ioSIOSET,     //SioSet
	//ioCLEARBUFF,  //ClearBuff
	//ioSIOCTRL,    //SioCtrl
	else if(str== "SioLink")
		pushTokenList(str,ioSIOLINK);
	else if(str== "SioGet")
		pushTokenList(str,ioSIOGET);
	else if(str== "SioSet")
		pushTokenList(str,ioSIOSET);
	else if(str== "ClearBuffer")
		pushTokenList(str,ioCLEARBUFF);
	else if(str== "SioCtrl")
		pushTokenList(str,ioSIOCTRL);

	/***************************************************
	* 函数名:加速度
	* 作者：林旭军
	* 创建日期：2017-1-16
	* 修改日期：2017-1-16
	***************************************************/
	else if(str== "ACCELERATION")
		pushTokenList(str,sdACCELERATION);
	/*********************end**************************/

	else{
		if(!AnalyzeFragmentString(str))
			return false;
	}
	return true;
}

bool wordscaner::AnalyzeFragmentString(string str)
{
	int n=str.length();
	if(str[0]=='0'&&str[1]=='.')//首字符为 0 的情形
	{
			for(int i=2;i<n;i++){//扫描其余字符
				if((str[i]<='z'&&str[i]>='a')||(str[i]<='Z'&&str[i]>='A')||(str[i]=='.'))
				{	reporterror(2);//如果出现了其他字母或小数点则报错
					return false;
				}
			}
			pushTokenList(str,sNUM);//扫描通过 字符串为0.shuzi
	}else if(str[0]=='0'&&n>1){
		reporterror(2);
		return false;
	}else if(str[0]<='9'&&str[0]>='0'){
		for(int i=1;i<n;i++){//扫描其余字符
			if((str[i]<='z'&&str[i]>='a')||(str[i]<='Z'&&str[i]>='A'))
			{
				reporterror(2);//如果出现了其他字母或小数点则报错
				return false;
			}
		}
		pushTokenList(str,sNUM);//扫描通过为数值
	}else{
		for(int i=1;i<n;i++){//扫描其余字符
			if(str[i]=='.')
			{	reporterror(2);//如果出现了小数点则报错
				return false;
			}
		}
		pushTokenList(str,sSTR);//扫描通过为纯字符串
	}
	return true;
}

bool wordscaner::AnalyzeChar(char chCurrent)
{
	string str;
	if(chCurrent=='='){
		if(strCurrent!=""){
			if(!AnalyzeString(strCurrent))//分析当前字符串片段
				return false;
			strCurrent="";//分析完之后置空
		}
		char c=getOneChar(filestring);
		if(c=='=')
		{
			str="==";
			pushTokenList(str,sEQ);
		}else{
			str="=";
			pushTokenList(str,sASSIGN);
			CurrentCharIndex--;
			status=START;
		}
	}
	else if(chCurrent== '+' ){
		if(strCurrent!=""){
			if(!AnalyzeString(strCurrent))//分析当前字符串片段
				return false;
			strCurrent="";//分析完之后置空
		}
		str="+";
		pushTokenList(str,sPLUS);
	}
	else if(chCurrent== '-' ){
		if(strCurrent!=""){
			if(!AnalyzeString(strCurrent))//分析当前字符串片段
				return false;
			strCurrent="";//分析完之后置空
		}
		str="-";
		pushTokenList(str,sMINUS);
	}
	else if(chCurrent== '*' ){
		if(strCurrent!=""){
			if(!AnalyzeString(strCurrent))//分析当前字符串片段
				return false;
			strCurrent="";//分析完之后置空
		}
		str="*";
		pushTokenList(str,sMULT);
	}
	else if(chCurrent=='/'){
		if(strCurrent!=""){
			if(!AnalyzeString(strCurrent))//分析当前字符串片段
				return false;
			strCurrent="";//分析完之后置空
		}
		str="/";
		pushTokenList(str,sDIV);
	}
	else if(chCurrent== '<' ){
		if(strCurrent!=""){
			if(!AnalyzeString(strCurrent))//分析当前字符串片段
				return false;
			strCurrent="";//分析完之后置空
		}
		char c=getOneChar(filestring);
		if(c=='=')
		{
			str="<=";
			pushTokenList(str,sLTOE);
		}else{
			str="<";
			pushTokenList(str,sLT);
			CurrentCharIndex--;
			status=START;
		}
	}
	else if(chCurrent=='>' ){
		if(strCurrent!=""){
			if(!AnalyzeString(strCurrent))//分析当前字符串片段
				return false;
			strCurrent="";//分析完之后置空
		}
		char c=getOneChar(filestring);
		if(c=='=')
		{
			str=">=";
			pushTokenList(str,sGTOE);
		}else{
			str=">";
			pushTokenList(str,sGT);
			CurrentCharIndex--;
			status=START;
		}
	}
	else if(chCurrent=='!' ){
		if(strCurrent!=""){
			if(!AnalyzeString(strCurrent))//分析当前字符串片段
				return false;
			strCurrent="";//分析完之后置空
		}
		char c=getOneChar(filestring);
		if(c=='=')
				{
					str="!=";
					pushTokenList(str,sUEQ);
				}else{
					reporterror(1);
					CurrentCharIndex--;
					status=START;
					return false;
			}
	}
	else if(chCurrent== ';' ){
		if(strCurrent!=""){
			if(!AnalyzeString(strCurrent))//分析当前字符串片段
				return false;
			strCurrent="";//分析完之后置空
		}
		str=";";
		pushTokenList(str,sSEMIC);
	}
	else if(chCurrent== ',' ){
		if(strCurrent!=""){
			if(!AnalyzeString(strCurrent))//分析当前字符串片段
				return false;
			strCurrent="";//分析完之后置空
		}
		str=",";
		pushTokenList(str,sCOMMA);
	}
	else if(chCurrent== '(' ){
		if(strCurrent!=""){
			if(!AnalyzeString(strCurrent))//分析当前字符串片段
				return false;
			strCurrent="";//分析完之后置空
		}
		str="(";
		pushTokenList(str,sLBRAC);
	}
	else if(chCurrent== ')' ){
		if(strCurrent!=""){
			if(!AnalyzeString(strCurrent))//分析当前字符串片段
				return false;
			strCurrent="";//分析完之后置空
		}
		str=")";
		pushTokenList(str,sRBRAC);
	}
	else if(chCurrent== '[' ){
		if(strCurrent!=""){
			if(!AnalyzeString(strCurrent))//分析当前字符串片段
				return false;
			strCurrent="";//分析完之后置空
		}
		str="[";
		pushTokenList(str,sLARRSYM);
	}
	else if(chCurrent==']' ){
		if(strCurrent!=""){
			if(!AnalyzeString(strCurrent))//分析当前字符串片段
				return false;
			strCurrent="";//分析完之后置空
		}
		str="]";
		pushTokenList(str,sRARRSYM);
	}
	else if(chCurrent=='{' ){
		if(strCurrent!=""){
			if(!AnalyzeString(strCurrent))//分析当前字符串片段
				return false;
			strCurrent="";//分析完之后置空
		}
		str="{";
		pushTokenList(str,sLBOUND);
	}
	else if(chCurrent== '}' ){
		if(strCurrent!=""){
			if(!AnalyzeString(strCurrent))//分析当前字符串片段
				return false;
			strCurrent="";//分析完之后置空
		}
		str="}";
		pushTokenList(str,sRBOUND);
	}
	else if(chCurrent== '\0' ){
		if(strCurrent!=""){
			if(!AnalyzeString(strCurrent))//分析当前字符串片段
				return false;
			strCurrent="";//分析完之后置空
		}
	}
	else if(chCurrent== '\n' ){
		if(strCurrent!=""){
			if(!AnalyzeString(strCurrent))//分析当前字符串片段
				return false;
			strCurrent="";//分析完之后置空
		}
		lineCount++;
	}
	else if(chCurrent== '\r' ){
		//考虑到系统差异，不作任何处理
	}
	else if(chCurrent== ' ' ){//空格
		if(strCurrent!=""){
			if(!AnalyzeString(strCurrent))//分析当前字符串片段
				return false;
			strCurrent="";//分析完之后置空
		}
	}
	else if(chCurrent== '\t' ){//tab
		if(strCurrent!=""){
			if(!AnalyzeString(strCurrent))//分析当前字符串片段
				return false;
			strCurrent="";//分析完之后置空
		}
	}
	else if((chCurrent<='9'&&chCurrent>='0' )||(chCurrent== '.' )||(chCurrent>='a'&&chCurrent<= 'z')||(chCurrent>='A'&&chCurrent<= 'Z')||(chCurrent== '_' )){
		strCurrent+=chCurrent;
	}
	else{
		reporterror(1);
		return false;
	}
	return true;
}
void wordscaner::pushTokenList(string str,M_TokenType tt)
{
	Token token;
	token.lineno=lineCount;
	token.m_TokenType=tt;
	token.tokenString=str;
	TokenList.push_back(token);
}
void wordscaner::reporterror(int errorflag)
{
#ifdef _WINDOWS_VS_
	 //输出框报错
	char lineNum[10];
	sprintf(lineNum,"%d",lineCount);
	string str_lnum=lineNum;


	if(errorflag==0){
		p_OutputWnd->ShowMessageInDeubugWnd("OPEN FILE ERROR!!! ");
	}
	if(errorflag==1){
		string message="ERROR!!! UNDEFINED SYMBOL! line: "+ str_lnum;
		p_OutputWnd->ShowMessageInDeubugWnd(message);
	}
	if(errorflag==2){
		string message="ERROR!!! BAD STRING!!  line: "+ str_lnum;
		p_OutputWnd->ShowMessageInDeubugWnd(message);
	}
	if(errorflag==3){
		p_OutputWnd->ShowMessageInDeubugWnd("空文档！");
	}
	return;
#else
	if(errorflag==0){
		cout<<"OPEN FILE ERROR!!! "<<endl;
	}
	if(errorflag==1){
		cout<<"ERROR!!! UNDEFINED SYMBOL!"<<"line: "<<lineCount<<'\t'<<chCurrent;
	}
	if(errorflag==2){
		cout<<"ERROR!!! BAD STRING!!"<<"line: "<<lineCount<<'\t'<<strCurrent;
	}
	if(errorflag==3){
		cout<<"void file!"<<endl;
	}
	return;
#endif
}

void wordscaner::IgnoreOthers(string str)
{

}
//////////////////////////////////////////////////////////////////////////////////////////////////////编译平台
#ifdef _WINDOWS_VS_
wordscaner::wordscaner(CRobotAppApp *theApp) {

	this->theApp=theApp;
	//CMainFrame *p_MainFrame;
	CMainFrame *p_MainFrame = (CMainFrame*)theApp->GetMainWnd();//获得程序主框架
	p_OutputWnd=p_MainFrame->GetOutputWnd();

	// TODO Auto-generated constructor stub
	filestring="";
	strCurrent="";
	CurrentCharIndex=0;
	lineCount=1;
	chCurrent='\0';
	status=WAIT;

	TokenList.clear();
}
#else
wordscaner::wordscaner() {

	// TODO Auto-generated constructor stub
	filestring="";
	strCurrent="";
	CurrentCharIndex=0;
	lineCount=1;
	chCurrent='\0';
	status=WAIT;

	TokenList.clear();
}
#endif




