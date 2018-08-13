//#include "../stdafx.h"

#include "Parse.h"
#include<iostream>
#include<fstream>
#include<stdlib.h>
#include<stdio.h>

#ifdef _WINDOWS_VS_
#include"..\RobotAppDoc.h"
#include"..\RobotAppView.h"
#include "..\FAction\F3DDefaultAction.h"
#endif

#include"stdio.h"

using namespace std;

Parse::Parse(void)
{
	nTokenList=0;//标识列表元素个数
	currentTokenIndex=0;
	root=NULL;
	root=new treeNode;
	//currentNode=new treeNode;
	end=NULL;

	Complete=false;
	parameterMOVC=pNone;
	parameterMOVL=pNone;
	parameterMOVJ=pNone;

	scaner=NULL;
	currentToken.lineno=1;
	currentToken.tokenString="";
	currentToken.m_TokenType=sASSIGN;

	funcIndex=0;
}
Parse::Parse(string path)
{
	nTokenList=0;//标识列表元素个数
	currentTokenIndex=0;
	root=NULL;
	root=new treeNode;
	//currentNode=new treeNode;
	end=NULL;

	Complete=false;
	parameterMOVC=pNone;
	parameterMOVL=pNone;
	parameterMOVJ=pNone;

	scaner=NULL;
	currentToken.lineno=1;
	currentToken.tokenString="";
	currentToken.m_TokenType=sASSIGN;

	funcIndex=0;

	this->path=path;
	int pos = -1;
	pos=path.find_last_of('/');
	if (pos==-1)
	{
		this->fileName=path;
	}
	else
	{
		this->fileName=path.substr(pos + 1);
	}

}
Parse::~Parse(void)
{
	//clearTree(root);
	//if(root !=NULL)
	//{
	//	delete root;
	//	root=NULL;
	//}
}
void Parse::clearTree(treeNode* root)
{
	treeNode *ls,*rs;
	if(root!=NULL)
	{
		ls=root->firstChild;
		rs=root->sibling;
		if(ls!=NULL)
		{
			if(ls->firstChild==NULL&&ls->sibling==NULL)
			{
				delete ls;
				ls=NULL;
			}
			else
			{
				clearTree(ls);
			}
		}
		if(rs!=NULL)
		{
			if(rs->firstChild==NULL&&rs->sibling==NULL)
			{
				delete rs;
				rs=NULL;
			}
			else
			{
				clearTree(rs);
			}
		}
	}
}
/**************************************************************************************************\
                                      语法树相关函数
\**************************************************************************************************/
bool Parse::BuildParseTree()
{
	if (!WordsScan())
	{
		return false;
	}
	Complete=false;
	getOneToken();
	if(match(sBEGIN)){
		root=makeNode(lastToken,ROOT);
	}
	else{
		reporterror("缺少程序起始符号 BEGIN !");
		return false;
	}
	if(scaner->TokenList.at(nTokenList-1).m_TokenType!=sEND){
		reporterror("缺少程序终止符号 END !");
		return false;
	}

	while(currentToken.m_TokenType!=sEND){
		if(!maketreeNode(currentToken,root))
			return false;
	}

	treeNode* currentNode;//指向最末的语法类型父节点
	currentNode=root;
	if(currentNode->firstChild!=NULL)
		currentNode=root->firstChild;
	while(currentNode->sibling!=NULL)
		currentNode=currentNode->sibling;
	currentNode->sibling=makeNode(currentToken,END);

	Complete=true;
#ifdef _WINDOWS_VS_
	p_OutputWnd->ShowMessageInDeubugWnd("语法树构建完成！");
#else
	cout<<"语法树构建完成！"<<endl;
#endif
	return true;
}
bool Parse::maketreeNode(Token t,treeNode* root)
{
	treeNode* currentNode;//指向最末的语法类型父节点
	currentNode=root;
	if(currentNode->firstChild!=NULL)
		currentNode=root->firstChild;
	while(currentNode->sibling!=NULL)
		currentNode=currentNode->sibling;

	treeNode *p_node=new treeNode;
	if(!p_node)
	{
		reporterror("Out of Space!");
		return false;
	}
	switch (t.m_TokenType)
	{
	/***************************************************
	* 函数名:加速度
	* 作者：林旭军
	* 创建日期：2017-1-16
	* 修改日期：2017-1-16
	***************************************************/
	case sdACCELERATION:
	/*********************end**************************/

	case sdBOOL:
	case sdINT:
	case sdDOUBLE:
	case sdSTRING:
	case sdPOSIT:
	case sdTOOL:
	case sdJOINT:
	case sdSPEED:
	case sdDIO:
	case sdSIO://////////////////////////////////////////////////////////////////////////////////////////声明
		p_node=makeNode(t,DeclarationT);
		if(currentNode==root)
		{
			root->firstChild=p_node;
		}else{
			currentNode->sibling=p_node;
		}
		if(!declaration(p_node,root))//是否构成声明语法
			return false;
		if(currentToken.m_TokenType!=sSEMIC)//是否具有语法终结符
		{
			reporterror("声明语句缺少终止符号 “;”");
			return false;
		}
		else
			getOneToken();
		break;
	case sSTR://///////////////////////////////////////////////////////////////////////////////////////////赋值
		p_node=TraversingTreeForDeclaration(root,currentToken.tokenString);
		if(p_node!=NULL){
			p_node=makeNode(currentToken,ValuationT);
			if(!valuation(p_node))
				return false;
			currentNode->sibling=p_node;//声明过必有至少一个类型节点
		}else{
			reporterror("出现未声明且不知意图的字符串！");
			return false;
		}
		if(currentToken.m_TokenType!=sSEMIC)//语句终结判定放在这里来执行
		{
			reporterror("赋值语句缺少终止符号 “;”");
			return false;
		}
		else
			getOneToken();
		break;
	case slIF:
		p_node=makeNode(t,IfT);
		if(currentNode==root)
		{
			root->firstChild=p_node;
		}else{
			currentNode->sibling=p_node;
		}
		if(!logic_if(p_node))
			return false;

		if(currentToken.m_TokenType!=sSEMIC)//语句终结判定放在这里来执行//////////////////////////////////test
		{}	//reporterror("赋值语句缺少终止符号 “;”");
		else
			getOneToken();
		break;
	case slWHILE:
		p_node=makeNode(t,WhileT);
		if(currentNode==root)
		{
			root->firstChild=p_node;
		}else{
			currentNode->sibling=p_node;
		}
		if(!logic_while(p_node))
			return false;

		if(currentToken.m_TokenType!=sSEMIC)//语句终结判定放在这里来执行//////////////////////////////////test
		{}	//reporterror("赋值语句缺少终止符号 “;”");
		else
			getOneToken();
		break;
	case sBREAK:
		reporterror("无效的break语句！");
		return false;
	case sCONTINUE:
		reporterror("无效的continue语句！");
		return false;
	//seMOVL,           // MOVL
	//seMOVC,           // MOVC
	//seMOVJ,           // MOVJ
	//seDisaPower,	  //DisablePower
	//seEnaPower,       //EnablePower
	//sePause,
	//seRestart,
	//seWaitEnd,
	//ioDIOLINK,    //DioLink
	//ioSIOLINK,    //SioLink
	case seMOVJ:
	case seMOVL:
	case seMOVC:
	case seDisaPower:
	case seEnaPower:
	case sePause:
	case seRestart:
	case seDelay:
	case seWaitEnd:
	case ioDIOLINK:
	case ioSIOLINK:
		p_node=makeNode(currentToken,NoReFUNC);
		if (!nodeNoReFunc(p_node,currentToken.m_TokenType))
		{
			return false;
		}
		currentNode->sibling=p_node;//如果声明过，树枝节点肯定不为空,currentNode一定不是根节点
		currentNode=currentNode->sibling;
		if(currentToken.m_TokenType!=sSEMIC)//语句终结判定放在这里来执行//////////////////////////////////test
		{
			reporterror("赋值语句缺少终止符号 “;”");
			return false;
		}
		else
			getOneToken();
		break;
	case slELSE:
		reporterror("ELSE紧跟IF之后不允许用“;”分开");
		return false;
	case BLENDS:
		p_node=makeNode(currentToken,BLENDST);
		if(currentNode==root)
		{
			root->firstChild=p_node;
		}else{
			currentNode->sibling=p_node;
		}
		if(!nodeBlends(p_node))
			return false;
		if(currentToken.m_TokenType!=sSEMIC)//语句终结判定放在这里来执行//////////////////////////////////test
		{
			reporterror("赋值语句缺少终止符号 “;”");
			return false;
		}
		else
			getOneToken();
		break;
	default:
		reporterror("语法有误，检测是否能够构成语句!");
		return false;
		//break;
	}
	return true;
}
bool Parse::makeStatementNode(Token t)
{
	treeNode* currentNode;//指向最末的语法类型父节点
	currentNode=root;
	if(currentNode->firstChild!=NULL)
		currentNode=root->firstChild;
	while(currentNode->sibling!=NULL)
		currentNode=currentNode->sibling;

	treeNode *p_node=new treeNode;
	if(!p_node)
	{
		reporterror("Out of Space!");
	}
	switch (t.m_TokenType)
	{//逻辑体内部不能有声明语句
	case sSTR://///////////////////////////////////////////////////////////////////////////////////////////赋值
		p_node=TraversingTreeForDeclaration(root,currentToken.tokenString);
		if(p_node!=NULL){
			p_node=makeNode(currentToken,ValuationT);
			if(!valuation(p_node))
				return false;
			currentNode->sibling=p_node;//声明过必有至少一个类型节点
		}else{
			reporterror("出现未声明且不知意图的字符串！");
			return false;
		}
		if(currentToken.m_TokenType!=sSEMIC)//语句终结判定放在这里来执行
		{
			reporterror("赋值语句缺少终止符号 “;”");
			return false;
		}
		else
			getOneToken();
		break;
	case slIF:
		p_node=makeNode(t,IfT);
		if(currentNode==root)
		{
			root->firstChild=p_node;
		}else{
			currentNode->sibling=p_node;
		}
		if(!logic_if(p_node))
			return false;

		if(currentToken.m_TokenType!=sSEMIC)//语句终结判定放在这里来执行//////////////////////////////////test
		{}	//reporterror("赋值语句缺少终止符号 “;”");
		else
			getOneToken();
		break;
	case slWHILE:
		p_node=makeNode(t,WhileT);
		if(currentNode==root)
		{
			root->firstChild=p_node;
		}else{
			currentNode->sibling=p_node;
		}
		if(!logic_while(p_node))
			return false;

		if(currentToken.m_TokenType!=sSEMIC)//语句终结判定放在这里来执行//////////////////////////////////test
		{}//	reporterror("赋值语句缺少终止符号 “;”");
		else
			getOneToken();
		break;
	case sBREAK:
		p_node=makeNode(currentToken,BreakT);
		currentNode->sibling=p_node;
		currentNode=currentNode->sibling;
		getOneToken();
		if(currentToken.m_TokenType!=sSEMIC)
		{
			reporterror("函数语句缺少终止符号 “;”");
			return false;
		}else
			getOneToken();
		break;
	case sCONTINUE:
		p_node=makeNode(currentToken,ContinueT);
		currentNode->sibling=p_node;
		currentNode=currentNode->sibling;
		getOneToken();
		if(currentToken.m_TokenType!=sSEMIC)
		{
			reporterror("函数语句缺少终止符号 “;”");
			return false;
		}else
			getOneToken();
		break;
	//seMOVL,           // MOVL
	//seMOVC,           // MOVC
	//seMOVJ,           // MOVJ
	//seDisaPower,	  //DisablePower
	//seEnaPower,       //EnablePower
	//sePause,
	//seRestart,
	//seWaitEnd,
	//ioDIOLINK,    //DioLink
	//ioSIOLINK,    //SioLink
	case seMOVJ:
	case seMOVL:
	case seMOVC:
	case seDisaPower:
	case seEnaPower:
	case sePause:
	case seRestart:
	case seWaitEnd:
	case seDelay:
	case ioDIOLINK:
	case ioSIOLINK:
		p_node=makeNode(currentToken,NoReFUNC);
		if (!nodeNoReFunc(p_node,currentToken.m_TokenType))
		{
			return false;
		}
		currentNode->sibling=p_node;//如果声明过，树枝节点肯定不为空,currentNode一定不是根节点
		currentNode=currentNode->sibling;
		if(currentToken.m_TokenType!=sSEMIC)//语句终结判定放在这里来执行//////////////////////////////////test
		{
			reporterror("赋值语句缺少终止符号 “;”");
			return false;
		}
		else
			getOneToken();
		break;
	case slELSE:
		reporterror("ELSE紧跟IF之后不允许用“;”分开");
		return false;
		break;
	case BLENDS:
		reporterror("BLENDS属性应定义在全局!");
		return false;
	default:
		reporterror("多余语法类型错误!");
		return false;
		//break;
	}
	return true;
}
bool Parse::StatementComplete()
{
	treeNode* cNode;//指向最末的语法类型父节点
	cNode=root;
	if(cNode->firstChild!=NULL)
		cNode=root->firstChild;
	while(cNode->sibling!=NULL)
		cNode=cNode->sibling;

	if(currentToken.m_TokenType!=sLBOUND){
		reporterror("缺少逻辑体执行左边界“{”");
		return false;
	}
	else{
		cNode->sibling=makeNode(currentToken,StatementLeftT);
		getOneToken();
		while(currentToken.m_TokenType!=sRBOUND)
		{
			if(currentToken.m_TokenType==sEND){
				reporterror("缺少逻辑体执行右边界“}”,不完整的程序！！！");
				return false;
			}
			if(!makeStatementNode(currentToken))
				return false;
		}
		treeNode* currentNode;//指向最末的语法类型父节点
		currentNode=root;
		if(currentNode->firstChild!=NULL)
			currentNode=root->firstChild;
		while(currentNode->sibling!=NULL)
			currentNode=currentNode->sibling;

		currentNode->sibling=makeNode(currentToken,StatementRightT);
		getOneToken();
	}
	return true;
}
bool Parse::declaration(treeNode* Node,treeNode* root)
{
	treeNode* p_temp;
	getOneToken();//当前token为声明类型后一个

	if(match(sSTR)){//声明后面第一个Token为字符串，当前token游标指向字符串后面一个
		if(RenameError(root,lastToken.tokenString))//检测是否重名
			return false;
	}else{
		reporterror("声明变量格式有误！");//否则报错语法树生成失败
		return false;
	}
	switch(currentToken.m_TokenType){
	case sCOMMA:// 当前 ,
		if(Node->firstChild!=NULL){//如果已经声明一个及以上
			p_temp=Node->firstChild;
			while(p_temp->sibling!=NULL)
				p_temp=p_temp->sibling;
			p_temp->sibling=makeNode(lastToken,DeclarationT);
		}else{//声明第一个变量
			Node->firstChild=makeNode(lastToken,DeclarationT);
		}
		if(!declaration(Node,root))
			return false;
		break;
	case sASSIGN:// 当前 =
	{
		treeNode* currentNode;//指向最末的语法类型父节点
		currentNode=root;
		if(currentNode->firstChild!=NULL)
			currentNode=root->firstChild;
		while(currentNode->sibling!=NULL)
			currentNode=currentNode->sibling;

		if(Node->firstChild!=NULL){//如果已经声明一个及以上
			p_temp=Node->firstChild;
			while(p_temp->sibling!=NULL)
				p_temp=p_temp->sibling;
			p_temp->sibling=makeNode(lastToken,DeclarationT);
		}else{//声明第一个变量
			Node->firstChild=makeNode(lastToken,DeclarationT);
		}

		treeNode* p_node=makeNode(lastToken,ValuationT);
		if(!valuation_tasking(p_node,Node->token.m_TokenType))
			return false;

		currentNode->sibling=p_node;

		if(currentToken.m_TokenType==sCOMMA)
		{	if(!declaration(Node,root))
				return false;
		}
		break;
	}
	default://遇到其它符号皆终止
		if(Node->firstChild!=NULL){//如果已经声明一个及以上
			p_temp=Node->firstChild;
			while(p_temp->sibling!=NULL)
				p_temp=p_temp->sibling;
			p_temp->sibling=makeNode(lastToken,DeclarationT);
		}else{//声明第一个变量
			Node->firstChild=makeNode(lastToken,DeclarationT);
		}
		break;
	}
	return true;
}
bool Parse::valuation(treeNode *Node)
{
	getOneToken();
	if(currentToken.m_TokenType!=sASSIGN){
		reporterror("未知意图的字符串，变量赋值子树归约失败！");//否则报错赋值语法树生成失败
		return false;
	}
	treeNode *p_forType=TraversingTreeForDeclaration(root,Node->token.tokenString);//根据声明token的类型来确定赋值任务
	if(!valuation_tasking(Node,p_forType->token.m_TokenType))
		return false;
	return true;
}
bool Parse::valuation_tasking(treeNode* Node,M_TokenType type)
{
	switch(type){
	case sdINT:
	case sdDOUBLE:
		if(!valuation_general(Node))
			return false;
		break;
	case sdBOOL:
		if(!valuation_bool(Node))
			return false;
		break;
	case sdSTRING:
		if(!valuation_string(Node))
			return false;
		break;
	case sdPOSIT:
	case sdJOINT:
		if(!valuation_posit(Node))
			return false;
		break;
	case sdTOOL:
		if(!valuation_tool(Node))
			return false;
		break;
	case sdSPEED:
		if(!valuation_speed(Node))
			return false;
		break;

	/***************************************************
	* 函数名:加速度
	* 作者：林旭军
	* 创建日期：2017-1-16
	* 修改日期：2017-1-16
	***************************************************/
	case sdACCELERATION:
		if(!valuation_acceleration(Node))
			return false;
		break;
	/*********************end**************************/

	case sdDIO:
		if(!valuation_dio(Node))
			return false;
		break;
	case sdSIO:
		if(!valuation_string(Node))
			return false;
		break;
	default:
		reporterror("不能赋值的变量");
		return false;
		//break;
	}
	return true;
}
bool Parse::valuation_general(treeNode* Node)
{
	//表达式归约过程
	vector<Token> opt_Token;
	vector<Token> postfixToken;
	nextstatus=First;
	//**************************************************************从中缀(infix)表达式到后缀(postfix)表达式的转换//
	getOneToken();
	funcIndex=0;
	funcAddress.clear();
	if(!transfor_infix_to_postfix(opt_Token,postfixToken))
		return false;
	//**************************************************************从中缀(infix)表达式到后缀(postfix)表达式的转换//
	//(由逆波兰记法 reverse Polish notation)构建表达式树
	Node->firstChild=build_postfix_tree(postfixToken);
	replaceTrueFuncOnLeft(Node->firstChild,Node);
	return true;
}
void Parse::replaceTrueFuncOnLeft(treeNode *node,treeNode*father)
{
	if (node==NULL)
	{
		return ;
	}
	if (node->nodetype==REFUNC)
	{
		if (node->sibling!=NULL)
		{
			if (node->sibling->nodetype==REFUNC)
			{
				unsigned int pos=atoi((node->sibling->token.tokenString).c_str());
				delete node->sibling;
				node->sibling=funcAddress.at(pos);
			}
			else
			{
				replaceTrueFuncOnLeft(node->sibling->firstChild,node->sibling);
			}
		}
		unsigned int pos=atoi((node->token.tokenString).c_str());
		funcAddress.at(pos)->sibling=node->sibling;
		delete father->firstChild;
		father->firstChild=funcAddress.at(pos);
	}
	else
	{
		if (node->sibling!=NULL)
		{
			if (node->sibling->nodetype==REFUNC)
			{
				unsigned int pos=atoi((node->sibling->token.tokenString).c_str());
				delete node->sibling;
				node->sibling=funcAddress.at(pos);
			}
			else
			{
				replaceTrueFuncOnLeft(node->sibling->firstChild,node->sibling);
			}
		}
		replaceTrueFuncOnLeft(node->firstChild,node);
	}

}
bool Parse::aidTransElement(vector<Token> & opt_Token,vector<Token> & postfixToken)
{
	//	ioDIOGET,     //DioGet
	//	ioDIOSET,     //DioSet
	//	ioDIOSTATUS,  //DioStatus
	//	ioSIOGET,     //SioGet
	//	ioSIOSET,     //SioSet
	//	ioCLEARBUFF,  //ClearBuff
	//	ioSIOCTRL,    //SioCtrl
	if(currentToken.m_TokenType==ioDIOGET||currentToken.m_TokenType==ioDIOSET||currentToken.m_TokenType==ioDIOSTATUS||
		currentToken.m_TokenType==ioSIOGET||currentToken.m_TokenType==ioSIOSET||currentToken.m_TokenType==ioCLEARBUFF||
		currentToken.m_TokenType==ioSIOCTRL)
	{
		if (!build_returnfunc(currentToken))
		{
			reporterror("表达式中所包含的函数归约失败...请修正！");
			return false;
		}
		Token t;
		t.lineno=currentToken.lineno;
		t.m_TokenType=RFUNC;
		char temp[16];
		sprintf(temp,"%d",funcIndex++);
		t.tokenString=temp;

		postfixToken.push_back(t);
		nextstatus=Opera;
		getOneToken();///////////////////////////////////////////////////////////////////
		if(!transfor_infix_to_postfix(opt_Token,postfixToken))
			return false;
	}else if(currentToken.m_TokenType==sSTR)
	{
		bool test;
		test=TraversingTreeValOrNot(root,currentToken.tokenString);
		if(test==false)
		{
			reporterror("表达式包含未赋值的变量！");
			return false;
		}else{
			postfixToken.push_back(currentToken);
			nextstatus=Opera;
			getOneToken();///////////////////////////////////////////////////////////////////
			if(!transfor_infix_to_postfix(opt_Token,postfixToken))
				return false;
		}
	}else if(currentToken.m_TokenType==sNUM){
		postfixToken.push_back(currentToken);
		nextstatus=Opera;
		getOneToken();///////////////////////////////////////////////////////////////////
		if(!transfor_infix_to_postfix(opt_Token,postfixToken))
			return false;
	}else if(currentToken.m_TokenType==sLBRAC){
		opt_Token.push_back(currentToken);
		nextstatus=Num;
		getOneToken();///////////////////////////////////////////////////////////////////
		if(!transfor_infix_to_postfix(opt_Token,postfixToken))
			return false;
	}else if(currentToken.m_TokenType==sMINUS){
		if (opt_Token.size()>1)
		{
			Token temp=opt_Token.at(opt_Token.size()-1);
			if(temp.m_TokenType!=sLBRAC){
				reporterror("赋值语句出错，负号“-”出现在不正确的位置！");
				return false;
			}
		}
		getOneToken();
		if(currentToken.m_TokenType==sSTR){
			bool test;
			test=TraversingTreeValOrNot(root,currentToken.tokenString);
			if(test==false)
			{
				reporterror("表达式包含未赋值的变量！");
				return false;
			}else{
				Token t1,t2;
				t1.lineno=currentToken.lineno;
				t1.tokenString="0";
				t1.m_TokenType=sNUM;
				postfixToken.push_back(t1);

				t2.lineno=currentToken.lineno;
				t2.tokenString="-";
				t2.m_TokenType=sMINUS;

				opt_Token.push_back(t2);

				postfixToken.push_back(currentToken);
				nextstatus=Opera;
				getOneToken();///////////////////////////////////////////////////////////////////
				if(!transfor_infix_to_postfix(opt_Token,postfixToken))
					return false;
			}
		}else if(currentToken.m_TokenType==sNUM){
			Token t1,t2;
			t1.lineno=currentToken.lineno;
			t1.tokenString="0";
			t1.m_TokenType=sNUM;
			postfixToken.push_back(t1);

			t2.lineno=currentToken.lineno;
			t2.tokenString="-";
			t2.m_TokenType=sMINUS;

			opt_Token.push_back(t2);

			postfixToken.push_back(currentToken);
			nextstatus=Opera;
			getOneToken();///////////////////////////////////////////////////////////////////
			if(!transfor_infix_to_postfix(opt_Token,postfixToken))
				return false;
		}else{
			reporterror("赋值语句出错，负号“-”右边必须有可负的值！");
			return false;
		}
	}else{
		reporterror("表达式有误！");
		return false;
	}
	return true;
}
bool Parse::build_returnfunc(Token cToken)
{
	switch (cToken.m_TokenType)
	{
		//	ioDIOGET,     //DioGet
		//	ioDIOSET,     //DioSet
		//	ioDIOSTATUS,  //DioStatus
		//	ioSIOGET,     //SioGet
		//	ioSIOSET,     //SioSet
		//	ioCLEARBUFF,  //ClearBuff
		//	ioSIOCTRL,    //SioCtrl
	case ioDIOGET:
		if(!reductionIODioGetFunc())
		{
			reporterror("DioGet函数归约失败！");
			return false;
		}
		break;
	case ioDIOSET:
		if(!reductionIODioSetFunc())
		{
			reporterror("DioSet函数归约失败！");
			return false;
		}
		break;
	case ioDIOSTATUS:
		if(!reductionIODioStatusFunc())
		{
			reporterror("DioStatus函数归约失败！");
			return false;
		}
		break;
	case ioSIOGET:
		if(!reductionIOSioGetFunc())
		{
			reporterror("SioGet函数归约失败！");
			return false;
		}
		break;
	case ioSIOSET:
		if(!reductionIOSioSetFunc())
		{
			reporterror("SioSet函数归约失败！");
			return false;
		}
		break;
	case ioCLEARBUFF:
		if(!reductionIOClearBuffFunc())
		{
			reporterror("ClearBuff函数归约失败！");
			return false;
		}
		break;
	case ioSIOCTRL:
		if(!reductionIOSioCtrlFunc())
		{
			reporterror("SioCtrl函数归约失败！");
			return false;
		}
		break;
	default:
		reporterror("该函数未定义...");
		return false;
	}
	return true;
}
bool Parse::reductionIODioGetFunc()
{
	treeNode *pFather=makeNode(currentToken,REFUNC);
	getOneToken();
	if (currentToken.m_TokenType!=sLBRAC)//"("
	{
		reporterror("缺少函数列表左包络符“(”!");
		return false;
	}
	getOneToken();
	if (currentToken.m_TokenType!=sSTR)//DIO
	{
		reporterror("函数参数有误!");
		return false;
	}
	treeNode *p_node=TraversingTreeForDeclaration(root,currentToken.tokenString);
	if(p_node==NULL){
		reporterror("使用了未定义的参数！");
		return false;
	};
	if(p_node->token.m_TokenType!=sdDIO&&p_node->token.m_TokenType!=sdINT){
		reporterror("参数应为DIO或者INT类型！");
		return false;
	};
	p_node=TraversingTreeForValNode(root,currentToken.tokenString,currentToken.m_TokenType);
	if(p_node==NULL){
		reporterror("函数使用了未赋值的参数!");
		return false;
	}
	getOneToken();
	if (currentToken.m_TokenType!=sRBRAC)//")"
	{
		reporterror("缺少函数列表右包络符“(”!");
		return false;
	}
	pFather->firstChild=makeNode(lastToken,ValT);
	funcAddress.push_back(pFather);
	return true;
}
bool Parse::reductionIODioSetFunc()
{
	treeNode *pFather=makeNode(currentToken,REFUNC);
	getOneToken();
	if (currentToken.m_TokenType!=sLBRAC)//"("
	{
		reporterror("缺少函数列表左包络符“(”!");
		return false;
	}
	getOneToken();
	if (currentToken.m_TokenType!=sSTR)//第一个参数
	{
		reporterror("函数参数有误!");
		return false;
	}
	treeNode *p_node=TraversingTreeForDeclaration(root,currentToken.tokenString);
	if(p_node==NULL){
		reporterror("使用了未定义的参数！");
		return false;
	};
	if(p_node->token.m_TokenType!=sdDIO&&p_node->token.m_TokenType!=sdINT){
		reporterror("函数第一个参数应为DIO或者INT类型！");
		return false;
	};
	p_node=TraversingTreeForValNode(root,currentToken.tokenString,currentToken.m_TokenType);
	if(p_node==NULL){
		reporterror("函数使用了未赋值的参数!");
		return false;
	}
	pFather->firstChild=makeNode(currentToken,ValT);
	getOneToken();
	if (currentToken.m_TokenType!=sCOMMA)//“,”
	{
		reporterror("函数参数列表缺少间隔符号“,”!");
		return false;
	}
	getOneToken();
	if (currentToken.m_TokenType!=sSTR)//第二个参数
	{
		reporterror("函数参数有误!");
		return false;
	}
	p_node=TraversingTreeForDeclaration(root,currentToken.tokenString);
	if(p_node==NULL){
		reporterror("使用了未定义的参数！");
		return false;
	};
	if(p_node->token.m_TokenType!=sdDIO&&p_node->token.m_TokenType!=sdINT){
		reporterror("参数应为DIO或者INT类型！");
		return false;
	};
	p_node=TraversingTreeForValNode(root,currentToken.tokenString,currentToken.m_TokenType);
	if(p_node==NULL){
		reporterror("函数使用了未赋值的参数!");
		return false;
	}
	getOneToken();
	if (currentToken.m_TokenType!=sRBRAC)//")"
	{
		reporterror("缺少函数列表右包络符“)”!");
		return false;
	}
	pFather->firstChild->sibling=makeNode(lastToken,ValT);
	funcAddress.push_back(pFather);
	return true;
}
bool Parse::reductionIODioStatusFunc()
{
	treeNode *pFather=makeNode(currentToken,REFUNC);
	getOneToken();
	if (currentToken.m_TokenType!=sLBRAC)//"("
	{
		reporterror("缺少函数列表左包络符“(”!");
		return false;
	}
	getOneToken();
	if (currentToken.m_TokenType!=sSTR)//DIO
	{
		reporterror("函数参数有误!");
		return false;
	}
	treeNode *p_node=TraversingTreeForDeclaration(root,currentToken.tokenString);
	if(p_node==NULL){
		reporterror("使用了未定义的参数！");
		return false;
	};
	if(p_node->token.m_TokenType!=sdDIO&&p_node->token.m_TokenType!=sdINT){
		reporterror("参数应为DIO或者INT类型！");
		return false;
	};
	p_node=TraversingTreeForValNode(root,currentToken.tokenString,currentToken.m_TokenType);
	if(p_node==NULL){
		reporterror("函数使用了未赋值的参数!");
		return false;
	}
	getOneToken();
	if (currentToken.m_TokenType!=sRBRAC)//")"
	{
		reporterror("缺少函数列表右包络符“)”!");
		return false;
	}
	pFather->firstChild=makeNode(lastToken,ValT);
	funcAddress.push_back(pFather);
	return true;
}
bool Parse::reductionIOSioGetFunc()
{
	treeNode *pFather=makeNode(currentToken,REFUNC);
	getOneToken();
	if (currentToken.m_TokenType!=sLBRAC)//"("
	{
		reporterror("缺少函数列表左包络符“(”!");
		return false;
	}
	getOneToken();
	if (currentToken.m_TokenType!=sSTR)//SIO
	{
		reporterror("函数参数有误!");
		return false;
	}
	treeNode *p_node=TraversingTreeForDeclaration(root,currentToken.tokenString);
	if(p_node==NULL){
		reporterror("使用了未定义的参数！");
		return false;
	};
	if(p_node->token.m_TokenType!=sdSIO&&p_node->token.m_TokenType!=sdSTRING){
		reporterror("参数应为SIO或者STRING类型！");
		return false;
	};
	p_node=TraversingTreeForValNode(root,currentToken.tokenString,currentToken.m_TokenType);
	if(p_node==NULL){
		reporterror("函数使用了未赋值的参数!");
		return false;
	}
	getOneToken();
	if (currentToken.m_TokenType!=sRBRAC)//")"
	{
		reporterror("缺少函数列表右包络符“)”!");
		return false;
	}
	pFather->firstChild=makeNode(lastToken,ValT);
	funcAddress.push_back(pFather);
	return true;
}
bool Parse::reductionIOSioSetFunc()
{
	treeNode *pFather=makeNode(currentToken,REFUNC);
	getOneToken();
	if (currentToken.m_TokenType!=sLBRAC)//"("
	{
		reporterror("缺少函数列表左包络符“(”!");
		return false;
	}
	getOneToken();
	if (currentToken.m_TokenType!=sSTR)//第一个参数
	{
		reporterror("函数参数有误!");
		return false;
	}
	treeNode *p_node=TraversingTreeForDeclaration(root,currentToken.tokenString);
	if(p_node==NULL){
		reporterror("使用了未定义的参数！");
		return false;
	};
	if(p_node->token.m_TokenType!=sdSIO&&p_node->token.m_TokenType!=sdSTRING){
		reporterror("函数第一个参数应为SIO或者STRING类型！");
		return false;
	};
	p_node=TraversingTreeForValNode(root,currentToken.tokenString,currentToken.m_TokenType);
	if(p_node==NULL){
		reporterror("函数使用了未赋值的参数!");
		return false;
	}
	pFather->firstChild=makeNode(currentToken,ValT);
	getOneToken();
	if (currentToken.m_TokenType!=sCOMMA)//“,”
	{
		reporterror("函数参数列表缺少间隔符号“,”!");
		return false;
	}
	getOneToken();
	if (currentToken.m_TokenType!=sSTR)//第二个参数
	{
		reporterror("函数参数有误!");
		return false;
	}
	p_node=TraversingTreeForDeclaration(root,currentToken.tokenString);
	if(p_node==NULL){
		reporterror("使用了未定义的参数！");
		return false;
	};
	/***************************************************
	* 函数名:I/O值传入
	* 作者：林旭军
	* 创建日期：2017-2-13
	* 修改日期：2017-2-13
	***************************************************/
	if(p_node->token.m_TokenType!=sdINT/*&&p_node->token.m_TokenType!=sdINT*/){
		reporterror("第二个参数应为INT类型！");
		return false;
	};
	/*********************end**************************/
	/*if(p_node->token.m_TokenType!=sdSIO){
		reporterror("第二个参数应为SIO类型！");
		return false;
	}*/
	p_node=TraversingTreeForValNode(root,currentToken.tokenString,currentToken.m_TokenType);
	if(p_node==NULL){
		reporterror("函数使用了未赋值的参数!");
		return false;
	}
	getOneToken();
	if (currentToken.m_TokenType!=sRBRAC)//")"
	{
		reporterror("缺少函数列表右包络符“)”!");
		return false;
	}
	pFather->firstChild->sibling=makeNode(lastToken,ValT);
	funcAddress.push_back(pFather);
	return true;
}
bool Parse::reductionIOClearBuffFunc()
{
	treeNode *pFather=makeNode(currentToken,REFUNC);
	getOneToken();
	if (currentToken.m_TokenType!=sLBRAC)//"("
	{
		reporterror("缺少函数列表左包络符“(”!");
		return false;
	}
	getOneToken();
	if (currentToken.m_TokenType!=sSTR)//SIO
	{
		reporterror("函数参数有误!");
		return false;
	}
	treeNode *p_node=TraversingTreeForDeclaration(root,currentToken.tokenString);
	if(p_node==NULL){
		reporterror("使用了未定义的参数！");
		return false;
	};
	if(p_node->token.m_TokenType!=sdSIO&&p_node->token.m_TokenType!=sdSTRING){
		reporterror("参数应为SIO或者STRING类型！");
		return false;
	};
	p_node=TraversingTreeForValNode(root,currentToken.tokenString,currentToken.m_TokenType);
	if(p_node==NULL){
		reporterror("函数使用了未赋值的参数!");
		return false;
	}
	getOneToken();
	if (currentToken.m_TokenType!=sRBRAC)//")"
	{
		reporterror("缺少函数列表右包络符“)”!");
		return false;
	}
	pFather->firstChild=makeNode(lastToken,ValT);
	funcAddress.push_back(pFather);
	return true;
}
bool Parse::reductionIOSioCtrlFunc()
{
	treeNode *pFather=makeNode(currentToken,REFUNC);
	getOneToken();
	if (currentToken.m_TokenType!=sLBRAC)//"("
	{
		reporterror("缺少函数列表左包络符“(”!");
		return false;
	}
	getOneToken();
	if (currentToken.m_TokenType!=sSTR)//第一个参数
	{
		reporterror("函数参数有误!");
		return false;
	}
	treeNode *p_node=TraversingTreeForDeclaration(root,currentToken.tokenString);
	if(p_node==NULL){
		reporterror("使用了未定义的参数！");
		return false;
	};
	if(p_node->token.m_TokenType!=sdSIO){
		reporterror("函数第一个参数应为SIO类型！");
		return false;
	};
	p_node=TraversingTreeForValNode(root,currentToken.tokenString,currentToken.m_TokenType);
	if(p_node==NULL){
		reporterror("函数使用了未赋值的参数!");
		return false;
	}
	pFather->firstChild=makeNode(currentToken,ValT);
	getOneToken();
	if (currentToken.m_TokenType!=sCOMMA)//“,”
	{
		reporterror("函数参数列表缺少间隔符号“,”!");
		return false;
	}
	getOneToken();
	if (currentToken.m_TokenType!=sSTR)//第二个参数
	{
		reporterror("函数参数有误!");
		return false;
	}
	p_node=TraversingTreeForDeclaration(root,currentToken.tokenString);
	if(p_node==NULL){
		reporterror("使用了未定义的参数！");
		return false;
	};
	if(p_node->token.m_TokenType!=sdSTRING){
		reporterror("第二个参数应为STRING类型！");
		return false;
	};
	p_node=TraversingTreeForValNode(root,currentToken.tokenString,currentToken.m_TokenType);
	if(p_node==NULL){
		reporterror("函数使用了未赋值的参数!");
		return false;
	}
	getOneToken();
	if (currentToken.m_TokenType!=sRBRAC)//")"
	{
		reporterror("缺少函数列表右包络符“)”!");
		return false;
	}
	pFather->firstChild->sibling=makeNode(lastToken,ValT);
	funcAddress.push_back(pFather);
	return true;
}
bool Parse::transfor_infix_to_postfix(vector<Token>& opt_Token,vector<Token>& postfixToken)
{
	//getOneToken();
	switch(nextstatus){
	case First:
		if (!aidTransElement(opt_Token,postfixToken))
		{
			reporterror("赋值语句有误，“=”右边必须有正确的表达式！");
			return false;
		}
		break;
	case Num:
		if (!aidTransElement(opt_Token,postfixToken))
		{
			reporterror("赋值表达式有误！");
			return false;
		}
		break;
	case Opera:
		if(currentToken.m_TokenType==sPLUS){// +

			if(opt_Token.size()!=0){
				Token t=opt_Token.at(opt_Token.size()-1);
				if(t.m_TokenType!=sLBRAC){
					opt_Token.pop_back();
					opt_Token.push_back(currentToken);
					postfixToken.push_back(t);
				}else{
					opt_Token.push_back(currentToken);
				}
			}else{
				opt_Token.push_back(currentToken);
			}
			nextstatus=Num;
			getOneToken();
			if(!transfor_infix_to_postfix(opt_Token,postfixToken))
				return false;

		}else if(currentToken.m_TokenType==sMINUS){// -

			if(opt_Token.size()!=0){
				Token t=opt_Token.at(opt_Token.size()-1);
				if(t.m_TokenType!=sLBRAC){
					opt_Token.pop_back();
					opt_Token.push_back(currentToken);
					postfixToken.push_back(t);
				}else{
					opt_Token.push_back(currentToken);
				}
			}else{
				opt_Token.push_back(currentToken);
			}
			nextstatus=Num;
			getOneToken();///////////////////////////////////////////////////////////////////
			if(!transfor_infix_to_postfix(opt_Token,postfixToken))
				return false;

		}else if(currentToken.m_TokenType==sMULT){// *

			if(opt_Token.size()!=0){
				Token t=opt_Token.at(opt_Token.size()-1);
				if(t.m_TokenType!=sLBRAC&&t.m_TokenType!=sPLUS&&t.m_TokenType!=sMINUS){
					opt_Token.pop_back();
					opt_Token.push_back(currentToken);
					postfixToken.push_back(t);
				}else{
					opt_Token.push_back(currentToken);
				}
			}else{
				opt_Token.push_back(currentToken);
			}
			nextstatus=Num;
			getOneToken();///////////////////////////////////////////////////////////////////
			if(!transfor_infix_to_postfix(opt_Token,postfixToken))
				return false;

		}else if(currentToken.m_TokenType==sDIV){// /

			if(opt_Token.size()!=0){
				Token t=opt_Token.at(opt_Token.size()-1);
				if(t.m_TokenType!=sLBRAC&&t.m_TokenType!=sPLUS&&t.m_TokenType!=sMINUS){
					opt_Token.pop_back();
					opt_Token.push_back(currentToken);
					postfixToken.push_back(t);
				}else{
					opt_Token.push_back(currentToken);
				}
			}else{
				opt_Token.push_back(currentToken);
			}
			nextstatus=Num;
			getOneToken();///////////////////////////////////////////////////////////////////
			if(!transfor_infix_to_postfix(opt_Token,postfixToken))
				return false;

		}else if(currentToken.m_TokenType==sRBRAC){// )

			if(opt_Token.size()!=0){
				Token t=opt_Token.at(opt_Token.size()-1);
				if(t.m_TokenType==sLBRAC){
					//reporterror("多余的“()”！");
					//return false;
					opt_Token.pop_back();
				}else
				{
					while(t.m_TokenType!=sLBRAC)
					{
						opt_Token.pop_back();
						postfixToken.push_back(t);
						if(opt_Token.size()!=0){
							t=opt_Token.at(opt_Token.size()-1);
						}else{
							reporterror("缺少“(”与“)”配对！");
							return false;
						}
					}
					opt_Token.pop_back();
				}

			}else{
				reporterror("多余的 “)” ！");
				return false;
			}
			nextstatus=Opera;
			getOneToken();///////////////////////////////////////////////////////////////////
			if(!transfor_infix_to_postfix(opt_Token,postfixToken))
				return false;

		}else{//////////////////////////////////////////////////////////////标记
			Token t;
			while(opt_Token.size()!=0)
			{
				t=opt_Token.at(opt_Token.size()-1);
				if(t.m_TokenType==sLBRAC){
					reporterror("缺少“)”与“(”配对！");
					return false;
				}else{
					opt_Token.pop_back();
					postfixToken.push_back(t);
					//getOneToken();///////////////////////////////////////////////////////////////////
				}
			}
		}
		break;
	default:
		reporterror("赋值操作异常！");
		return false;
		//break;
	}
	return true;
}
treeNode* Parse::build_postfix_tree(vector<Token>& postfixToken)
{

	vector<treeNode> tempList;
	treeNode* p_node;
	Token p_temp;
	int n_postList=postfixToken.size();
	for(int i=0;i<n_postList;i++)
	{
		p_temp=postfixToken.at(i);
		if(p_temp.m_TokenType==sSTR)
		{
			p_node=makeNode(p_temp,ValT);
			tempList.push_back(*p_node);
		}else if(p_temp.m_TokenType==sNUM)
		{
			p_node=makeNode(p_temp,NumT);
			tempList.push_back(*p_node);
		}else if(p_temp.m_TokenType==RFUNC)
		{
			p_node=makeNode(p_temp,REFUNC);
			tempList.push_back(*p_node);
		}else{
			p_node=makeNode(p_temp,OperatorT);

			treeNode *p_tempnode1=new treeNode;
			treeNode *p_tempnode2=new treeNode;

			*p_tempnode2=tempList.at(tempList.size()-1);
			tempList.pop_back();

			*p_tempnode1=tempList.at(tempList.size()-1);
			tempList.pop_back();

			p_node->firstChild=p_tempnode1;
			p_tempnode1->sibling=p_tempnode2;
			tempList.push_back(*p_node);
		}
	}
	treeNode* p=new treeNode;
	*p=tempList.at(0);
	return p;
}
bool Parse::valuation_bool(treeNode* Node)
{
	getOneToken();
	treeNode *p_node;
	if(match(sTRUE)||match(sFALSE)){
		p_node=makeNode(lastToken,ValuationT);
		Node->firstChild=p_node;
	}
	else if(lastToken.m_TokenType==seIsPowered)
	{
		Token temp=lastToken;
		if(match(sLBRAC))
		{
			if (match(sRBRAC))
			{
				p_node=makeNode(temp,EIsPowered);
				return true;
			}
			else
			{
				reporterror("函数应带参数列表“()”！");
			}
		}
		else
		{
			reporterror("函数应带参数列表“()”！");
		}
	}
	else if(lastToken.m_TokenType==seIsSettled)
	{
		Token temp=lastToken;
		if(match(sLBRAC))
		{
			if (match(sRBRAC))
			{
				p_node=makeNode(temp,EIsSettled);
				return true;
			}
			else
			{
				reporterror("函数应带参数列表“()”！");
			}
		}
		else
		{
			reporterror("函数应带参数列表“()”！");
		}
	}
	else
	{
		reporterror("BOOL型变量只接受返回“TRUE”和“FALSE”两个值！");
		return false;
	}
	return true;
}
bool Parse::valuation_speed(treeNode* Node)
{
	getOneToken();
	treeNode *p_node;
	if(match(sNUM)){
		p_node=makeNode(lastToken,NumT);
		Node->firstChild=p_node;
	}else{
		reporterror("SPEED型速度变量只接受具体数值！");
		return false;
	}
	return true;
}
/***************************************************
* 函数名:加速度
* 作者：林旭军
* 创建日期：2017-1-16
* 修改日期：2017-1-16
***************************************************/
bool Parse::valuation_acceleration(treeNode *Node)
{
	getOneToken();
	treeNode *p_node;
	if(match(sNUM))
	{
		p_node=makeNode(lastToken,NumT);
		Node->firstChild=p_node;
	}
	else
	{
		reporterror("ACCELERATION型加速度变量只接受具体数值！");
		return false;
	}
	return true;
}
/*********************end**************************/

bool Parse::valuation_dio(treeNode* Node)
{
	getOneToken();
	treeNode *p_node;
	if(match(sNUM)){
		p_node=makeNode(lastToken,NumT);
		Node->firstChild=p_node;
	}else{
		reporterror("DIO型变量只接受具体数值！");
		return false;
	}
	return true;
}
bool Parse::valuation_string(treeNode* Node)
{
	getOneToken();
	treeNode *p_node;
	if(match(sSTR)){
		treeNode *test=TraversingTreeForDeclaration(root,lastToken.tokenString);
		if(test!=NULL){
			reporterror("不能使用已声明过的字符串！");
			return false;
		}
		else{
			p_node=makeNode(lastToken,ValuationT);
			Node->firstChild=p_node;
		}
	}else{
		reporterror("STRING或SIO型变量值只接受字符串！");
		return false;
	}
	return true;
}
bool Parse::valuation_posit(treeNode* Node)
{
	positStatus=None;
	if(!val_posit_aidfuc(Node))
		return false;
	return true;
}
bool Parse::val_posit_aidfuc(treeNode* Node)
{
	switch(positStatus){
	case None:
		getOneToken();
		if(currentToken.m_TokenType==sLARRSYM){
			positStatus=NUM;
		    if(!val_posit_aidfuc(Node))
				return false;
		}else{
			reporterror("POSIT或JOINT赋值应该以“[”与“]”包围值！");
			return false;
		}
		break;
	case NUM:
		getOneToken();
		if(currentToken.m_TokenType==sNUM){
			int count=0;
			if(Node->firstChild==NULL){
				treeNode *p_node=makeNode(currentToken,ValuationT);
				Node->firstChild=p_node;
			}else{
				treeNode *p_temp=Node->firstChild;
				count++;
				while(p_temp->sibling!=NULL){
					count++;
					p_temp=p_temp->sibling;
				}
				treeNode *p_node=makeNode(currentToken,ValuationT);
				p_temp->sibling=p_node;
			}
			if(count<5){
				positStatus=COMMA;
				if(!val_posit_aidfuc(Node))
					return false;
			}else{
				positStatus=Close;
				if(!val_posit_aidfuc(Node))
					return false;
			}
		}else if(currentToken.m_TokenType==sMINUS){
			getOneToken();
			Token t;
			t.lineno=currentToken.lineno;
			t.m_TokenType=currentToken.m_TokenType;
			t.tokenString=lastToken.tokenString+currentToken.tokenString;
			if(currentToken.m_TokenType==sNUM){
				int count=0;
				if(Node->firstChild==NULL){
					treeNode *p_node=makeNode(t,ValuationT);
					Node->firstChild=p_node;
				}else{
					treeNode *p_temp=Node->firstChild;
					count++;
					while(p_temp->sibling!=NULL){
						count++;
						p_temp=p_temp->sibling;
					}
					treeNode *p_node=makeNode(t,ValuationT);
					p_temp->sibling=p_node;
				}
				if(count<5){
					positStatus=COMMA;
					if(!val_posit_aidfuc(Node))
						return false;
				}else{
					positStatus=Close;
					if(!val_posit_aidfuc(Node))
						return false;
				}
			}else{
				reporterror("POSIT或JOINT数据类型赋值时,负号“-”后面应为数值字符串！");
				return false;
			}
		}else{
			reporterror("POSIT或JOINT数据类型赋值错误,值应为数值字符串！");
			return false;
		}
		break;
	case COMMA:
		getOneToken();
		if(currentToken.m_TokenType==sCOMMA){
			positStatus=NUM;
			if(!val_posit_aidfuc(Node))
				return false;
		}else{
			reporterror("POSIT或JOINT数据类型赋值应该以“,”间隔！");
			return false;
		}
		break;
	case Close:
		getOneToken();
		if(currentToken.m_TokenType==sRARRSYM){
			positStatus=None;
			getOneToken();
		}else{
			reporterror("POSIT数据类型赋值缺少“]”与“[”配对！");
			return false;
		}
		break;
	default:
		reporterror("POSIT类型赋值发生未可知的错误！ ");
		return false;
		break;
	}
	return true;
}

bool Parse::valuation_tool(treeNode* Node)
{
	toolStatus=None;
	if(!val_tool_aidfuc(Node))
		return false;
	return true;
}
bool Parse::val_tool_aidfuc(treeNode* Node)
{
	switch(toolStatus){
	case None:
		getOneToken();
		if(currentToken.m_TokenType==sLARRSYM){
			toolStatus=NUM;
		    val_tool_aidfuc(Node);
		}else{
			reporterror("TOOL赋值应该以“[”与“]”包围值！");
			return false;
		}
		break;
	case NUM:
		getOneToken();
		if(currentToken.m_TokenType==sNUM){
			int count=0;
			if(Node->firstChild==NULL){
				treeNode *p_node=makeNode(currentToken,ValuationT);
				Node->firstChild=p_node;
			}else{
				treeNode *p_temp=Node->firstChild;
				count++;
				while(p_temp->sibling!=NULL){
					count++;
					p_temp=p_temp->sibling;
				}
				treeNode *p_node=makeNode(currentToken,ValuationT);
				p_temp->sibling=p_node;
			}
			if(count<11){
				toolStatus=COMMA;
				if(!val_tool_aidfuc(Node))
					return false;
			}else{
				toolStatus=Close;
				if(!val_tool_aidfuc(Node))
					return false;
			}
		}else if(currentToken.m_TokenType==sMINUS){
			getOneToken();
			Token t;
			t.lineno=currentToken.lineno;
			t.m_TokenType=currentToken.m_TokenType;
			t.tokenString=lastToken.tokenString+currentToken.tokenString;
			if(currentToken.m_TokenType==sNUM){
				int count=0;
				if(Node->firstChild==NULL){
					treeNode *p_node=makeNode(t,ValuationT);
					Node->firstChild=p_node;
				}else{
					treeNode *p_temp=Node->firstChild;
					count++;
					while(p_temp->sibling!=NULL){
						count++;
						p_temp=p_temp->sibling;
					}
					treeNode *p_node=makeNode(t,ValuationT);
					p_temp->sibling=p_node;
				}
				if(count<5){
					toolStatus=COMMA;
					if(!val_tool_aidfuc(Node))
						return false;
				}else{
					toolStatus=Close;
					if(!val_tool_aidfuc(Node))
						return false;
				}
			}else{
				reporterror("TOOL数据类型赋值时,负号“-”后面应为数值字符串！");
				return false;
			}
		}else{
			reporterror("TOOL数据类型赋值错误,值应为数值字符串！");
			return false;
		}
		break;
	case COMMA:
		getOneToken();
		if(currentToken.m_TokenType==sCOMMA){
			toolStatus=NUM;
			if(!val_tool_aidfuc(Node))
				return false;
		}else{
			reporterror("TOOL数据类型赋值应该以“,”间隔！");
			return false;
		}
		break;
	case Close:
		getOneToken();
		if(currentToken.m_TokenType==sRARRSYM){
			toolStatus=None;
			getOneToken();
		}else{
			reporterror("TOOL数据类型赋值缺少“]”与“[”配对！");
			return false;
		}
		break;
	default:
		reporterror("TOOL类型赋值发生未可知的错误！ ");
		return false;
		break;
	}
	return true;
}
bool Parse::nodeMovJ(treeNode* Node)
{
	getOneToken();
	treeNode *p_node;
	switch(parameterMOVJ){
	case pNone:
		if(currentToken.m_TokenType==sLBRAC){
			parameterMOVJ=pPOSIT;
		    if(!nodeMovJ(Node))
				return false;
		}else{
			reporterror("MOVJ参数应该以“(”与“)”包围值！");
			return false;
		}
		break;
	case pPOSIT:
		if(currentToken.m_TokenType==sSTR){
			treeNode *p_node=TraversingTreeForDeclaration(root,currentToken.tokenString);
			if(p_node==NULL){
				reporterror("使用了未定义的参数！");
				return false;
			};
			if(p_node->token.m_TokenType!=sdPOSIT&&p_node->token.m_TokenType!=sdJOINT){
				reporterror("MOVJ第一个参数应为POSIT或者JOINT类型！");
				return false;
			};
			p_node=TraversingTreeForValNode(root,currentToken.tokenString,currentToken.m_TokenType);
			if(p_node==NULL){
				reporterror("使用了未赋值的POSIT或者JOINT变量！");
				return false;
			}
			//if(Node->firstChild==NULL){
			p_node=makeNode(currentToken,ValT);//第一个参数
			Node->firstChild=p_node;

			getOneToken();//检验下一个标识符号是否为 ,
			if(currentToken.m_TokenType!=sCOMMA){
				reporterror("参数列表应该以“,”间隔！");
				return false;
			}
			parameterMOVJ=pSPEED;//归约下一个参数
			if(!nodeMovJ(Node))
				return false;

		}else{
			reporterror("MOVJ参数类型错误！");
			return false;
		}
		break;
	case pSPEED:
		if(currentToken.m_TokenType==sSTR){
			p_node=TraversingTreeForDeclaration(root,currentToken.tokenString);
			if(p_node==NULL){
				reporterror("使用了未定义的参数！");
				return false;
			};
			if(p_node->token.m_TokenType!=sdSPEED){
				reporterror("MOVJ第二个参数应为SPEED类型！");
				return false;
			};
			p_node=TraversingTreeForValNode(root,currentToken.tokenString,currentToken.m_TokenType);
			if(p_node==NULL){
				reporterror("使用了未赋值的SPEED变量！");
				return false;
			}

			p_node=makeNode(currentToken,ValT);
			Node->firstChild->sibling=p_node;//第二个参数

			getOneToken();//检验下一个标识符号是否为 ,
			if(currentToken.m_TokenType!=sCOMMA){
				reporterror("参数列表应该以“,”间隔！");
				return false;
			}

			/***************************************************
			* 函数名:加速度
			* 作者：林旭军
			* 创建日期：2017-1-16
			* 修改日期：2017-1-16
			***************************************************/
			getOneToken();
			if(currentToken.m_TokenType==sSTR)//判断下一个参数是否是加速度
			{
				treeNode* p_node2=TraversingTreeForDeclaration(root,currentToken.tokenString);
				if(p_node2!=NULL&&p_node2->token.m_TokenType==sdACCELERATION)
					parameterMOVJ=pACCELERATION;//归约下一个参数为加速度
				else
					parameterMOVJ=pTOOL;//归约下一个参数为工具
				currentTokenIndex--;
			}
			/*********************end**************************/

			//parameterMOVJ=pTOOL;//归约下一个参数
			if(!nodeMovJ(Node))
				return false;

		}else{
			reporterror("MOVJ参数类型错误！");
			return false;
		}
		break;

	/***************************************************
	* 函数名:加速度
	* 作者：林旭军
	* 创建日期：2017-1-16
	* 修改日期：2017-1-16
	***************************************************/
	case pACCELERATION:
		if(currentToken.m_TokenType==sSTR)
		{
			p_node=TraversingTreeForDeclaration(root,currentToken.tokenString);
			if(NULL==p_node)
			{
				reporterror("使用了未定义的加速度参数！");
				return false;
			}
			if(sdACCELERATION!=p_node->token.m_TokenType)
			{
				reporterror("MOVJ第三个参数应为加速度类型！");
				return false;
			}
			p_node=TraversingTreeForValNode(root,currentToken.tokenString,currentToken.m_TokenType);
			if(NULL==p_node)
			{
				reporterror("使用了未赋值的ACCELERATION变量！");
				return false;
			}
			p_node=makeNode(currentToken,ValT);
			treeNode *curNode2;
			curNode2=Node->firstChild;
			while(curNode2->sibling!=NULL)
			{
				curNode2=curNode2->sibling;
			}
			curNode2->sibling=p_node;//加速度参数
			getOneToken();//检验下一个标识符号是否为 ,
			if(sCOMMA!=currentToken.m_TokenType)
			{
				reporterror("参数列表应该以“,”间隔！");
				return false;
			}
			parameterMOVL=pTOOL;//归约下一个参数
			if(!nodeMovL(Node))
				return false;
		}
		else
		{
			reporterror("MOVJ的ACCELERATION参数类型错误！");
			return false;
		}
		break;
	/*********************end**************************/

	case pTOOL:
		if(currentToken.m_TokenType==sSTR){
			p_node=TraversingTreeForDeclaration(root,currentToken.tokenString);
			if(p_node==NULL){
				reporterror("使用了未定义的参数！");
				return false;
			};
			if(p_node->token.m_TokenType!=sdTOOL){
				reporterror("MOVJ第三个参数应为TOOL类型！");
				return false;
			};
			p_node=TraversingTreeForValNode(root,currentToken.tokenString,currentToken.m_TokenType);
			if(p_node==NULL){
				reporterror("使用了未赋值的TOOL变量！");
				return false;
			}

			p_node=makeNode(currentToken,ValT);

			/***************************************************
			* 函数名:加速度
			* 作者：林旭军
			* 创建日期：2017-1-16
			* 修改日期：2017-1-16
			***************************************************/
			treeNode *curNode;
			curNode=Node->firstChild;
			while(curNode->sibling!=NULL)
			{
				curNode=curNode->sibling;
			}
			curNode->sibling=p_node;
			/*********************end**************************/

			//Node->firstChild->sibling->sibling=p_node;//第三个参数

			parameterMOVJ=pClose;
			if(!nodeMovJ(Node))
				return false;

		}else{
			reporterror("MOVJ参数类型错误！");
			return false;
		}
		break;
	case pClose:
		if(currentToken.m_TokenType==sRBRAC){
			parameterMOVJ=pNone;
			getOneToken();
		}else{
			reporterror("MOVJ函数缺少“)”与“(”配对！");
			return false;
		}
		break;
	default:
		reporterror("MOVJ函数发生未可知的错误！ ");
		return false;
		//break;
	}
	return true;
}
bool Parse::nodeMovL(treeNode* Node)
{
	getOneToken();
	treeNode *p_node;
	switch(parameterMOVL){
	case pNone:
		if(currentToken.m_TokenType==sLBRAC){
			parameterMOVL=pPOSIT;
		    if(!nodeMovL(Node))
				return false;
		}else{
			reporterror("MOVL参数应该以“(”与“)”包围值！");
			return false;
		}
		break;
	case pPOSIT:
		if(currentToken.m_TokenType==sSTR){
			p_node=TraversingTreeForDeclaration(root,currentToken.tokenString);
			if(p_node==NULL){
				reporterror("使用了未定义的参数！");
				return false;
			};
			if(p_node->token.m_TokenType!=sdPOSIT&&p_node->token.m_TokenType!=sdJOINT){
				reporterror("MOVL第一个参数应为POSIT或者JOINT类型！");
				return false;
			};
			p_node=TraversingTreeForValNode(root,currentToken.tokenString,currentToken.m_TokenType);
			if(p_node==NULL){
				reporterror("使用了未赋值的POSIT或者JOINT变量！");
				return false;
			}
			//if(Node->firstChild==NULL){
			p_node=makeNode(currentToken,ValT);//第一个参数
			Node->firstChild=p_node;

			getOneToken();//检验下一个标识符号是否为 ,
			if(currentToken.m_TokenType!=sCOMMA){
				reporterror("参数列表应该以“,”间隔！");
				return false;
			}
			parameterMOVL=pSPEED;//归约下一个参数
			if(!nodeMovL(Node))
				return false;

		}else{
			reporterror("MOVL参数类型错误！");
			return false;
		}
		break;
	case pSPEED:
		if(currentToken.m_TokenType==sSTR){
			p_node=TraversingTreeForDeclaration(root,currentToken.tokenString);
			if(p_node==NULL){
				reporterror("使用了未定义的参数！");
				return false;
			};
			if(p_node->token.m_TokenType!=sdSPEED){
				reporterror("MOVL第二个参数应为SPEED类型！");
				return false;
			};
			p_node=TraversingTreeForValNode(root,currentToken.tokenString,currentToken.m_TokenType);
			if(p_node==NULL){
				reporterror("使用了未赋值的SPEED变量！");
				return false;
			}
			//if(Node->firstChild==NULL){
			p_node=makeNode(currentToken,ValT);
			Node->firstChild->sibling=p_node;//第二个参数

			getOneToken();//检验下一个标识符号是否为 ,
			if(currentToken.m_TokenType!=sCOMMA){
				reporterror("参数列表应该以“,”间隔！");
				return false;
			}

			/***************************************************
			* 函数名:加速度
			* 作者：林旭军
			* 创建日期：2017-1-16
			* 修改日期：2017-1-16
			***************************************************/
			getOneToken();
			if(currentToken.m_TokenType==sSTR)//判断下一个参数是否是加速度
			{
				treeNode* p_node2=TraversingTreeForDeclaration(root,currentToken.tokenString);
				if(p_node2!=NULL&&p_node2->token.m_TokenType==sdACCELERATION)
					parameterMOVL=pACCELERATION;//归约下一个参数为加速度
				else
					parameterMOVL=pTOOL;//归约下一个参数为工具
				currentTokenIndex--;
			}
			/*********************end**************************/

			//parameterMOVL=pTOOL;//归约下一个参数
			if(!nodeMovL(Node))
				return false;

		}else{
			reporterror("MOVL参数类型错误！");
			return false;
		}
		break;

	/***************************************************
	* 函数名:加速度
	* 作者：林旭军
	* 创建日期：2017-1-16
	* 修改日期：2017-1-16
	***************************************************/
	case pACCELERATION:
		if(currentToken.m_TokenType==sSTR)
		{
			p_node=TraversingTreeForDeclaration(root,currentToken.tokenString);
			if(NULL==p_node)
			{
				reporterror("使用了未定义的加速度参数！");
				return false;
			}
			if(sdACCELERATION!=p_node->token.m_TokenType)
			{
				reporterror("MOVL第三个参数应为加速度类型！");
				return false;
			}
			p_node=TraversingTreeForValNode(root,currentToken.tokenString,currentToken.m_TokenType);
			if(NULL==p_node)
			{
				reporterror("使用了未赋值的ACCELERATION变量！");
				return false;
			}
			p_node=makeNode(currentToken,ValT);
			treeNode *curNode2;
			curNode2=Node->firstChild;
			while(curNode2->sibling!=NULL)
			{
				curNode2=curNode2->sibling;
			}
			curNode2->sibling=p_node;//加速度参数
			getOneToken();//检验下一个标识符号是否为 ,
			if(sCOMMA!=currentToken.m_TokenType)
			{
				reporterror("参数列表应该以“,”间隔！");
				return false;
			}
			parameterMOVL=pTOOL;//归约下一个参数
			if(!nodeMovL(Node))
				return false;
		}
		else
		{
			reporterror("MOVL的ACCELERATION参数类型错误！");
			return false;
		}
		break;
	/*********************end**************************/

	case pTOOL:
		if(currentToken.m_TokenType==sSTR){
			p_node=TraversingTreeForDeclaration(root,currentToken.tokenString);
			if(p_node==NULL){
				reporterror("使用了未定义的参数！");
				return false;
			};
			if(p_node->token.m_TokenType!=sdTOOL){
				reporterror("MOVL第三个参数应为TOOL类型！");
				return false;
			};
			p_node=TraversingTreeForValNode(root,currentToken.tokenString,currentToken.m_TokenType);
			if(p_node==NULL){
				reporterror("使用了未赋值的TOOL变量！");
				return false;
			}
			//if(Node->firstChild==NULL){
			p_node=makeNode(currentToken,ValT);

			/***************************************************
			* 函数名:加速度
			* 作者：林旭军
			* 创建日期：2017-1-16
			* 修改日期：2017-1-16
			***************************************************/
			treeNode *curNode;
			curNode=Node->firstChild;
			while(curNode->sibling!=NULL)
			{
				curNode=curNode->sibling;
			}
			curNode->sibling=p_node;
			/*********************end**************************/

			//Node->firstChild->sibling->sibling=p_node;//第三个参数

			parameterMOVL=pClose;
			if(!nodeMovL(Node))
				return false;

		}else{
			reporterror("MOVL参数类型错误！");
			return false;
		}
		break;
	case pClose:
		if(currentToken.m_TokenType==sRBRAC){
			parameterMOVL=pNone;
			getOneToken();
		}else{
			reporterror("MOVL函数缺少“)”与“(”配对！");
			return false;
		}
		break;
	default:
		reporterror("MOVL函数发生未可知的错误！ ");
		return false;
		//break;
	}
	return true;
}
bool Parse::nodeMovC(treeNode* Node)
{
	getOneToken();
	treeNode *p_node;
	switch(parameterMOVC){
	case pNone:
		if(currentToken.m_TokenType==sLBRAC){
			parameterMOVC=pPOSIT;
		    if(!nodeMovC(Node))
				return false;
		}else{
			reporterror("MOVC参数应该以“(”与“)”包围值！");
			return false;
		}
		break;
	case pPOSIT:
		if(currentToken.m_TokenType==sSTR){
			p_node=TraversingTreeForDeclaration(root,currentToken.tokenString);
			if(p_node==NULL){
				reporterror("使用了未定义的参数！");
				return false;
			};
			if(p_node->token.m_TokenType!=sdPOSIT&&p_node->token.m_TokenType!=sdJOINT){
				reporterror("MOVC第一个参数应为POSIT或者JOINT类型！");
				return false;
			};
			p_node=TraversingTreeForValNode(root,currentToken.tokenString,currentToken.m_TokenType);
			if(p_node==NULL){
				reporterror("使用了未赋值的POSIT或者JOINT变量！");
				return false;
			}

			p_node=makeNode(currentToken,ValT);//第一个参数
			Node->firstChild=p_node;

			getOneToken();//检验下一个标识符号是否为 ,
			if(currentToken.m_TokenType!=sCOMMA){
				reporterror("参数列表应该以“,”间隔！");
				return false;
			}
			parameterMOVC=pPOSIT2;//归约下一个参数
			if(!nodeMovC(Node))
				return false;

		}else{
			reporterror("MOVC参数类型错误！");
			return false;
		}
		break;
	case pPOSIT2:
		if(currentToken.m_TokenType==sSTR){
			treeNode *p_node=TraversingTreeForDeclaration(root,currentToken.tokenString);
			if(p_node==NULL){
				reporterror("使用了未定义的参数！");
				return false;
			};
			if(p_node->token.m_TokenType!=sdPOSIT&&p_node->token.m_TokenType!=sdJOINT){
				reporterror("MOVC第二个参数应为POSIT或者JOINT类型！");
				return false;
			};
			p_node=TraversingTreeForValNode(root,currentToken.tokenString,currentToken.m_TokenType);
			if(p_node==NULL){
				reporterror("使用了未赋值的POSIT或者JOINT变量！");
				return false;
			}
			p_node=makeNode(currentToken,ValT);//第二个参数
			Node->firstChild->sibling=p_node;

			getOneToken();//检验下一个标识符号是否为 ,
			if(currentToken.m_TokenType!=sCOMMA){
				reporterror("参数列表应该以“,”间隔！");
				return false;
			}
			parameterMOVC=pSPEED;//归约下一个参数
			if(!nodeMovC(Node))
				return false;

		}else{
			reporterror("MOVC参数类型错误！");
			return false;
		}
		break;
	case pSPEED:
		if(currentToken.m_TokenType==sSTR){
			p_node=TraversingTreeForDeclaration(root,currentToken.tokenString);
			if(p_node==NULL){
				reporterror("使用了未定义的参数！");
				return false;
			};
			if(p_node->token.m_TokenType!=sdSPEED){
				reporterror("MOVC第三个参数应为SPEED类型！");
				return false;
			};
			p_node=TraversingTreeForValNode(root,currentToken.tokenString,currentToken.m_TokenType);
			if(p_node==NULL){
				reporterror("使用了未赋值的SPEED变量！");
				return false;
			}

			p_node=makeNode(currentToken,ValT);
			Node->firstChild->sibling->sibling=p_node;//第三个参数

			getOneToken();//检验下一个标识符号是否为 ,
			if(currentToken.m_TokenType!=sCOMMA){
				reporterror("参数列表应该以“,”间隔！");
				return false;
			}

			/***************************************************
			* 函数名:加速度
			* 作者：林旭军
			* 创建日期：2017-1-16
			* 修改日期：2017-1-16
			***************************************************/
			getOneToken();
			if(currentToken.m_TokenType==sSTR)//判断下一个参数是否是加速度
			{
				treeNode* p_node2=TraversingTreeForDeclaration(root,currentToken.tokenString);
				if(p_node2!=NULL&&p_node2->token.m_TokenType==sdACCELERATION)
					parameterMOVC=pACCELERATION;//归约下一个参数为加速度
				else
					parameterMOVC=pTOOL;//归约下一个参数为工具
				currentTokenIndex--;
			}
			/*********************end**************************/

			//parameterMOVC=pTOOL;//归约下一个参数
			if(!nodeMovC(Node))
				return false;

		}else{
			reporterror("MOVC参数类型错误！");
			return false;
		}
		break;

	/***************************************************
	* 函数名:加速度
	* 作者：林旭军
	* 创建日期：2017-1-16
	* 修改日期：2017-1-16
	***************************************************/
	case pACCELERATION:
		if(currentToken.m_TokenType==sSTR)
		{
			p_node=TraversingTreeForDeclaration(root,currentToken.tokenString);
			if(NULL==p_node)
			{
				reporterror("使用了未定义的加速度参数！");
				return false;
			}
			if(sdACCELERATION!=p_node->token.m_TokenType)
			{
				reporterror("MOVC第三个参数应为加速度类型！");
				return false;
			}
			p_node=TraversingTreeForValNode(root,currentToken.tokenString,currentToken.m_TokenType);
			if(NULL==p_node)
			{
				reporterror("使用了未赋值的ACCELERATION变量！");
				return false;
			}
			p_node=makeNode(currentToken,ValT);
			treeNode *curNode2;
			curNode2=Node->firstChild;
			while(curNode2->sibling!=NULL)
			{
				curNode2=curNode2->sibling;
			}
			curNode2->sibling=p_node;//加速度参数
			getOneToken();//检验下一个标识符号是否为 ,
			if(sCOMMA!=currentToken.m_TokenType)
			{
				reporterror("参数列表应该以“,”间隔！");
				return false;
			}
			parameterMOVL=pTOOL;//归约下一个参数
			if(!nodeMovL(Node))
				return false;
		}
		else
		{
			reporterror("MOVC的ACCELERATION参数类型错误！");
			return false;
		}
		break;
	/*********************end**************************/

	case pTOOL:
		if(currentToken.m_TokenType==sSTR){
			p_node=TraversingTreeForDeclaration(root,currentToken.tokenString);
			if(p_node==NULL){
				reporterror("使用了未定义的参数！");
				return false;
			};
			if(p_node->token.m_TokenType!=sdTOOL){
				reporterror("MOVC第四个参数应为TOOL类型！");
				return false;
			};
			p_node=TraversingTreeForValNode(root,currentToken.tokenString,currentToken.m_TokenType);
			if(p_node==NULL){
				reporterror("使用了未赋值的TOOL变量！");
				return false;
			}

			p_node=makeNode(currentToken,ValT);

			/***************************************************
			* 函数名:加速度
			* 作者：林旭军
			* 创建日期：2017-1-16
			* 修改日期：2017-1-16
			***************************************************/
			treeNode *curNode;
			curNode=Node->firstChild;
			while(curNode->sibling!=NULL)
			{
				curNode=curNode->sibling;
			}
			curNode->sibling=p_node;
			/*********************end**************************/

			//Node->firstChild->sibling->sibling->sibling=p_node;//第四个参数

			parameterMOVC=pClose;
			if(!nodeMovC(Node))
				return false;

		}else{
			reporterror("MOVC参数类型错误！");
			return false;
		}
		break;
	case pClose:
		if(currentToken.m_TokenType==sRBRAC){
			parameterMOVC=pNone;
			getOneToken();
		}else{
			reporterror("MOVC函数缺少“)”与“(”配对！");
			return false;
		}
		break;
	default:
		reporterror("MOVC函数发生未可知的错误！ ");
		return false;
		//break;
	}
	return true;
}
bool Parse::logic_if(treeNode* Node)
{
	Node->firstChild=make_logic_BoolExpression();
	if(Node->firstChild==NULL)
		return false;
////////////////////////////////////////////////////////////////////////////////比较表达式归约结束
	if(!StatementComplete())
		return false;
///////////////////////////////////////////////////////////////////////////////有没有else
	if(currentToken.m_TokenType==slELSE){
		treeNode* currentNode;//指向最末的语法类型父节点
		currentNode=root;
		if(currentNode->firstChild!=NULL)
			currentNode=root->firstChild;
		while(currentNode->sibling!=NULL)
			currentNode=currentNode->sibling;

		currentNode->sibling=makeNode(currentToken,ElseT);
		getOneToken();
		if(!StatementComplete())
			return false;
	};
	return true;
}
treeNode* Parse::make_logic_BoolExpression()
{
	treeNode *p_node=new treeNode;
	getOneToken();
	if(currentToken.m_TokenType==sSTR/*||currentToken.m_TokenType==sNUM*/){
		if(TraversingTreeValOrNot(root,currentToken.tokenString)){
			p_node=make_logic_bool_expression_first();
		}else if(TraversingTree_othersValOrNot(root,currentToken.tokenString,sdBOOL)){
			p_node=make_logic_bool_expression_second();

		}else{
			reporterror("布尔表达式暂不支持此类型的比较！");
			return NULL;
		}
	}else if(currentToken.m_TokenType==sNUM){
		p_node=make_logic_bool_expression_first();
	}
	else if(currentToken.m_TokenType==sTRUE||currentToken.m_TokenType==sFALSE){
		p_node=make_logic_bool_expression_second();
	}else{
		reporterror("缺少必要的比较表达式");
		return NULL;
	}
	if(p_node==NULL){
		reporterror("比较表达式归约失败，请检查语句！");
		return NULL;
	}
	return p_node;
}
treeNode* Parse::make_logic_bool_expression_first()
{
	treeNode *p_node=new treeNode;
	treeNode *p_left;
	treeNode *p_right;
	vector<Token> opt_Token1;
	vector<Token> postfixToken1;
	nextstatus=First;
	//**************************************************************从中缀(infix)表达式到后缀(postfix)表达式的转换//
	transfor_infix_to_postfix(opt_Token1,postfixToken1);
	//**************************************************************从中缀(infix)表达式到后缀(postfix)表达式的转换//
	//(由逆波兰记法 reverse Polish notation)构建表达式i树
	p_left=build_postfix_tree(postfixToken1);

	if(currentToken.m_TokenType!=sLTOE&&currentToken.m_TokenType!=sGTOE&&currentToken.m_TokenType!=sLT&&currentToken.m_TokenType!=sGT
		&&currentToken.m_TokenType!=sEQ&&currentToken.m_TokenType!=sUEQ)
		reporterror("缺少比较表达式比较运算符！");
	else
		p_node=makeNode(currentToken,JudgeT);

	vector<Token> opt_Token2;
	vector<Token> postfixToken2;
	nextstatus=First;
	getOneToken();
	//**************************************************************从中缀(infix)表达式到后缀(postfix)表达式的转换//
	transfor_infix_to_postfix(opt_Token2,postfixToken2);
	//**************************************************************从中缀(infix)表达式到后缀(postfix)表达式的转换//
	if(postfixToken2.size()==0)
	{
		reporterror("布尔表达式右边必须要有可比较的对象！");
		return NULL;
	}
	//(由逆波兰记法 reverse Polish notation)构建表达式树
	p_right=build_postfix_tree(postfixToken2);

	if(p_left!=NULL&&p_right!=NULL){
		p_node->firstChild=p_left;
		p_left->sibling=p_right;
	}else{
		reporterror("比较表达式归约失败！");
	}
	return p_node;
}
treeNode* Parse::make_logic_bool_expression_second()
{
	treeNode *p_node=new treeNode;

	getOneToken();
	if(currentToken.m_TokenType==sEQ||currentToken.m_TokenType==sUEQ){
		p_node=makeNode(currentToken,JudgeT);
		treeNode *p_left;
		p_left=makeNode(lastToken,ValT);
		p_node->firstChild=p_left;
		getOneToken();
		if(currentToken.m_TokenType==sSTR){
			if(TraversingTreeValOrNot(root,currentToken.tokenString)){
				treeNode *p_right=makeNode(currentToken,ValT);
				p_left->sibling=p_right;

				getOneToken();

			}else{
				reporterror("使用了未赋值的变量");
			}
		}else if(currentToken.m_TokenType==sTRUE||currentToken.m_TokenType==sFALSE){
			treeNode *p_right=makeNode(currentToken,ValT);
			p_left->sibling=p_right;

			getOneToken();

		}else if(currentToken.m_TokenType==sNUM){
			treeNode *p_right=makeNode(currentToken,NumT);
			p_left->sibling=p_right;

			getOneToken();

		}else{
			reporterror("表达式右边应是能够被比较的对象！");
		}
	}else
		reporterror("布尔型比较表达式比较运算符只支持“==”和“!=”！");
	return p_node;
}
bool Parse::logic_while(treeNode* Node)
{
	Node->firstChild=make_logic_BoolExpression();
	if(Node->firstChild==NULL)
		return false;
////////////////////////////////////////////////////////////////////////////////比较表达式归约结束
	if(!StatementComplete())
		return false;
	return true;
}
bool Parse::nodeNoReFunc(treeNode* Node,M_TokenType type)
{
	//seMOVL,           // MOVL
	//seMOVC,           // MOVC
	//seMOVJ,           // MOVJ
	//seDisaPower,	  //DisablePower
	//seEnaPower,       //EnablePower
	//sePause,
	//seRestart,
	//seWaitEnd,
	//ioDIOLINK,    //DioLink
	//ioSIOLINK,    //SioLink
	switch (type)
	{
	case seMOVL:
		parameterMOVL=pNone;
		if(!nodeMovL(Node))
			return false;
		break;
	case seMOVJ:
		parameterMOVJ=pNone;
		if(!nodeMovJ(Node))
			return false;
		break;
	case seMOVC:
		parameterMOVC=pNone;
		if(!nodeMovC(Node))
			return false;
		break;
	case seDisaPower:
	case seEnaPower:
	case sePause:
	case seRestart:
	case seWaitEnd:
		if(!nodeNoListFunc(Node))
			return false;
		break;
	case seDelay:
		if(!nodeDelayFunc(Node))
			return false;
		break;
	case ioDIOLINK:
		if(!nodeIODioLinkFunc(Node))
			return false;
		break;
	case ioSIOLINK:
		if(!nodeIOSioLinkFunc(Node))
			return false;
		break;
	default:
		reporterror("该类型无法归约...");
		return false;
	}
	return true;
}
bool Parse::nodeNoListFunc(treeNode* Node)
{
	getOneToken();
	if(currentToken.m_TokenType==sLBRAC)
	{
		getOneToken();
		if(currentToken.m_TokenType!=sRBRAC)
		{
			reporterror("函数应该以“(”与“)”区别！");
			return false;
		}
		else
		{
			getOneToken();
		}
	}
	else
	{
		reporterror("函数应该以“(”与“)”区别！");
		return false;
	}
	return true;
}
bool Parse::nodeDelayFunc(treeNode* Node)
{
	getOneToken();
	if(currentToken.m_TokenType==sLBRAC)
	{
		getOneToken();
		if(currentToken.m_TokenType!=sNUM)
		{
			reporterror("函数参数为具体数值！");
			return false;
		}
		else
		{
			Node->firstChild=makeNode(currentToken,NumT);
		}
		getOneToken();
		if(currentToken.m_TokenType!=sRBRAC)
		{
			reporterror("函数应该以“(”与“)”区别！");
			return false;
		}
		getOneToken();

	}
	else
	{
		reporterror("函数应该以“(”与“)”区别！");
		return false;
	}
	return true;
}
bool Parse::nodeIODioLinkFunc(treeNode* Node)
{
	getOneToken();
	if (currentToken.m_TokenType!=sLBRAC)//"("
	{
		reporterror("缺少函数列表左包络符“(”!");
		return false;
	}
	getOneToken();
	if (currentToken.m_TokenType!=sSTR)//第一个参数
	{
		reporterror("函数参数有误!");
		return false;
	}
	treeNode *p_node=TraversingTreeForDeclaration(root,currentToken.tokenString);
	if(p_node==NULL){
		reporterror("使用了未定义的参数！");
		return false;
	};
	if(p_node->token.m_TokenType!=sdDIO/*&&p_node->token.m_TokenType!=sdINT*/){
		reporterror("函数第一个参数应为DIO类型！");
		return false;
	};
	p_node=TraversingTreeForValNode(root,currentToken.tokenString,currentToken.m_TokenType);
	if(p_node==NULL){
		reporterror("函数使用了未赋值的参数!");
		return false;
	}
	Node->firstChild=makeNode(currentToken,ValT);
	getOneToken();
	if (currentToken.m_TokenType!=sCOMMA)//“,”
	{
		reporterror("函数参数列表缺少间隔符号“,”!");
		return false;
	}
	getOneToken();
	if (currentToken.m_TokenType!=sSTR)//第二个参数
	{
		reporterror("函数参数有误!");
		return false;
	}
	p_node=TraversingTreeForDeclaration(root,currentToken.tokenString);
	if(p_node==NULL){
		reporterror("使用了未定义的参数！");
		return false;
	};
	if(p_node->token.m_TokenType!=sdDIO/*&&p_node->token.m_TokenType!=sdINT*/){
		reporterror("参数应为DIO类型！");
		return false;
	};
	p_node=TraversingTreeForValNode(root,currentToken.tokenString,currentToken.m_TokenType);
	if(p_node==NULL){
		reporterror("函数使用了未赋值的参数!");
		return false;
	}
	getOneToken();
	if (currentToken.m_TokenType!=sRBRAC)//")"
	{
		reporterror("缺少函数列表右包络符“)”!");
		return false;
	}
	Node->firstChild->sibling=makeNode(lastToken,ValT);
	getOneToken();
	return true;
}
bool Parse::nodeIOSioLinkFunc(treeNode* Node)
{
	getOneToken();
	if (currentToken.m_TokenType!=sLBRAC)//"("
	{
		reporterror("缺少函数列表左包络符“(”!");
		return false;
	}
	getOneToken();
	if (currentToken.m_TokenType!=sSTR)//第一个参数
	{
		reporterror("函数参数有误!");
		return false;
	}
	treeNode *p_node=TraversingTreeForDeclaration(root,currentToken.tokenString);
	if(p_node==NULL){
		reporterror("使用了未定义的参数！");
		return false;
	};
	if(p_node->token.m_TokenType!=sdSIO){
		reporterror("函数第一个参数应为SIO类型！");
		return false;
	};
	p_node=TraversingTreeForValNode(root,currentToken.tokenString,currentToken.m_TokenType);
	if(p_node==NULL){
		reporterror("函数使用了未赋值的参数!");
		return false;
	}
	Node->firstChild=makeNode(currentToken,ValT);
	getOneToken();
	if (currentToken.m_TokenType!=sCOMMA)//“,”
	{
		reporterror("函数参数列表缺少间隔符号“,”!");
		return false;
	}
	getOneToken();
	if (currentToken.m_TokenType!=sSTR)//第二个参数
	{
		reporterror("函数参数有误!");
		return false;
	}
	p_node=TraversingTreeForDeclaration(root,currentToken.tokenString);
	if(p_node==NULL){
		reporterror("使用了未定义的参数！");
		return false;
	};
	if(p_node->token.m_TokenType!=sdSIO){
		reporterror("参数应为SIO类型！");
		return false;
	};
	p_node=TraversingTreeForValNode(root,currentToken.tokenString,currentToken.m_TokenType);
	if(p_node==NULL){
		reporterror("函数使用了未赋值的参数!");
		return false;
	}
	getOneToken();
	if (currentToken.m_TokenType!=sRBRAC)//")"
	{
		reporterror("缺少函数列表右包络符“)”!");
		return false;
	}
	Node->firstChild->sibling=makeNode(lastToken,ValT);
	getOneToken();
	return true;
}
bool Parse::nodeBlends(treeNode* Node)
{
	getOneToken();
	treeNode *temp=new treeNode;
	if(currentToken.m_TokenType==sASSIGN)
	{
		getOneToken();

		if(currentToken.m_TokenType==sLARRSYM)//[
		{
			getOneToken();
		}
		else
		{
			reporterror("BlENDS属性赋值错误！");
			return false;
		}

		if(currentToken.m_TokenType==DEFAULT||currentToken.m_TokenType==SMOOTHTRA)
		{
			Node->firstChild=makeNode(currentToken,BLENDST);
			getOneToken();
		}
		else
		{
			reporterror("BlENDS属性只支持DEFAULT与SMOOTHTRA两种值！");
			return false;
		}

		if(currentToken.m_TokenType==sCOMMA)//,
		{
			getOneToken();
		}
		else
		{
			reporterror("BlENDS属性赋值错误，缺少属性值，以“,”间隔");
			return false;
		}

		if(currentToken.m_TokenType==sNUM)
		{
			Node->firstChild->sibling=makeNode(currentToken,BLENDST);
			getOneToken();
		}
		else
		{
			reporterror("BlENDS属性缺少属性值！");
			return false;
		}

		if(currentToken.m_TokenType==sRARRSYM)//]
		{
			getOneToken();
		}
		else
		{
			reporterror("BlENDS属性赋值错误！");
			return false;
		}
	}
	else
	{
		reporterror("BlENDS属性未赋值！");
		return false;
	}
	return true;
}
//-------------------------------------------------------------------------------------------------------endline

/**************************************************************************************************\
                                          工具函数
\**************************************************************************************************/
void Parse::getOneToken()
{
	if(currentTokenIndex<nTokenList){
		lastToken=currentToken;
		currentToken=scaner->TokenList.at(currentTokenIndex++);
	}else{
		reporterror("返回标识符错误，已到达标识符向量表尾！");
	}
}
bool Parse::match(M_TokenType type)
{
	if(currentToken.m_TokenType==type)
	{
		getOneToken();
		return true;
	}else{
		//reporterror("匹配失败！");
		return false;
	}
}
treeNode* Parse::makeNode(Token t,NodeType type)
{
	treeNode *p_node=new treeNode;
	if(!p_node)
	{
		reporterror("Out of Space!");
	}
	p_node->firstChild=NULL;
	p_node->token=t;
	p_node->nodetype=type;
	p_node->sibling=NULL;
	return p_node;
}
treeNode* Parse::TraversingTreeForDeclaration(treeNode* Root,string str)
{
	treeNode* currN=Root;
	treeNode* currStringN=NULL;
	if(Root==NULL)
		return NULL;
	else{
		currN=Root->firstChild;
	}
	while(currN!=NULL)
	{
		currStringN=currN->firstChild;
		while(currStringN!=NULL)
		{
			if(currStringN->token.tokenString==str)
				return currN;
			currStringN=currStringN->sibling;
		}
		currN=currN->sibling;
	}
	return NULL;
}
bool Parse::TraversingTreeValOrNot(treeNode* Root,string str)
{
	treeNode* currN=TraversingTreeForDeclaration(Root,str);
	if(currN==NULL)//是否已经声明
		return false;
	if(currN->token.m_TokenType==sdINT||currN->token.m_TokenType==sdDOUBLE)
	{
		currN=currN->sibling;
		while(currN!=NULL)
		{
			if(currN->nodetype==ValuationT&&currN->token.tokenString==str)
				return true;
			currN=currN->sibling;
		}
		return false;
	}
	else{
		return false;
	}
}
void Parse::reporterror(string x)
{

#ifdef _WINDOWS_VS_
	 //输出框报错
	char lineNum[10];
	sprintf(lineNum,"%d",currentToken.lineno);
	string str_lnum=lineNum;
	string message="ERROR!!! "+x+"  line: "+ str_lnum;
	p_OutputWnd->ShowMessageInDeubugWnd(message);
	return;
#else
	cout<<"ERROR!!!   "<<x<<"  line: "<<currentToken.lineno<<endl;
	system("pause");
	///////////////////////////////modified by xujinrong
	//exit(0);
#endif
}
bool Parse::RenameError(treeNode* Root,string str)
{
	if(TraversingTreeForDeclaration(Root,str)!=NULL)
	{
		reporterror("变量重复声明");
		return true;
	}
	return false;
}
treeNode* Parse::TraversingTreeForValNode(treeNode* Root,string str,M_TokenType type)
{
	treeNode *p_temp=Root;
	if(p_temp==NULL){
		return NULL;
	}else{
		p_temp=p_temp->firstChild;
	}
	while(p_temp!=NULL){
		if(p_temp->nodetype==ValuationT&&p_temp->token.tokenString==str&&p_temp->token.m_TokenType==type){
			return p_temp;
		}else{
			p_temp=p_temp->sibling;
		}
	}
	return NULL;
}
treeNode* Parse::TraversingTreeForMovNode(treeNode* Root,NodeType type)
{
	treeNode *p_temp=Root;
	if(p_temp==NULL){
		return NULL;
	}else{
		p_temp=p_temp->firstChild;
	}
	while(p_temp!=NULL){
		if(p_temp->nodetype==type){
			return p_temp;
		}else{
			p_temp=p_temp->sibling;
		}
	}
	return NULL;
}
bool Parse::TraversingTree_othersValOrNot(treeNode* Root,string str,M_TokenType type)
{
	treeNode* currN=TraversingTreeForDeclaration(Root,str);
	if(currN==NULL)//是否已经声明
		return false;
	if(currN->token.m_TokenType==type)
	{
		currN=currN->sibling;
		while(currN!=NULL)
		{
			if(currN->nodetype==ValuationT&&currN->token.tokenString==str)
				return true;
			currN=currN->sibling;
		}
		return false;
	}
	else{
		return false;
	}
}
////////////////////////////////////////////////////////////////////////////////////////////
void Parse::GetPathInfo(string path,string fileName)
{
	this->path=path;
	this->fileName=fileName;
}
bool Parse::WordsScan()
{
#ifdef _WINDOWS_VS_
	scaner=new wordscaner(theApp);
	if(scaner->Analyze(path)){
		nTokenList=scaner->TokenList.size();
		return true;
	}
	return false;
#else
	scaner=new wordscaner();
	scaner->Analyze(path);
	if(scaner->Analyze(path)){
		nTokenList=scaner->TokenList.size();
		return true;
	}
	return false;
#endif
}
