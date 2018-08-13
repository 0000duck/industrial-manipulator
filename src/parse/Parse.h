#pragma once

#ifndef _MYPARSE_
#define  _MYPARSE_


#include "wordscaner.h"
//#include "../MainFrm.h"

//class CCodeEditView;

using namespace std;

typedef enum{
	First=1,
	Num,
	Opera
}nextStatus;//表达式转换下一操作状态

typedef enum{
	None=1,
	NUM,
	COMMA,
	Close
}valStatus;//赋值状态

typedef enum{
	pNone=1,
	pPOSIT,
	pPOSIT2,
	pSPEED,

	/***************************************************
	* 函数名:加速度
	* 作者：林旭军
	* 创建日期：2017-1-16
	* 修改日期：2017-1-16
	***************************************************/
	pACCELERATION,
	/*********************end**************************/

	pTOOL,
	pClose
}parameterStatus;//函数参数状态

typedef enum{
	ROOT=1,//根
	END,//末

	//---------------------------------------------
	DeclarationT,//声明
	//---------------------------------------------
	ValuationT,//赋值
	ValT,
	NumT,
	OperatorT,

	IfT,
	JudgeT,
	StatementLeftT,//子逻辑体左边界
	StatementRightT,//子逻辑体右边界
	ElseT,

	WhileT,
	BreakT,
	ContinueT,

	//EMOVJ,
	//EMOVL,
	//EMOVC,
	//EDisaPower,
	//EEnaPower,
	//EPause,
	//ERestart,
	//EWaitEnd,
	EIsSettled,
	EIsPowered,
	//ExpressionT

	REFUNC,
	NoReFUNC,

	BLENDST

}NodeType;

struct sonNode{
	sonNode *child;
	sonNode *sibling;
	string str;
	M_TokenType type;//父节点Token类型
};

struct treeNode{
	treeNode *sibling;
	treeNode *firstChild;
	NodeType nodetype;
	Token token;//来自词法的标识
};

typedef enum{
	BDEFAULT,
	SmoothTra
}Blends;

class Parse
{
public:
	Parse(string path);
	Parse(void);
	~Parse(void);
protected:
	wordscaner *scaner;
	string path;
	string fileName;
public:
	bool WordsScan();
	void GetPathInfo(string path,string fileName);
	wordscaner *GetScanner(){return scaner;};
public:
	unsigned long nTokenList;//标识列表元素个数
	unsigned long currentTokenIndex;//当前标识元素在列表中的位置
	Token lastToken;//上一个标识符
	Token currentToken;//当前标识符


public://与语法树相关
	bool BuildParseTree();//构建语法树
	bool Complete;
protected:
	treeNode *root;//根节点
	treeNode *end;//末
	bool maketreeNode(Token t,treeNode* root);//创建树
	bool declaration(treeNode* Node,treeNode* root);//构建声明父节点
	bool valuation(treeNode* Node);//构建赋值父节点
	bool logic_if(treeNode* Node);//构建if逻辑体父节点
	bool logic_while(treeNode* Node);//构建while逻辑体父节点

	treeNode* make_logic_BoolExpression();
	treeNode* make_logic_bool_expression_first();//构建if逻辑体布尔表达式 树
	treeNode* make_logic_bool_expression_second();//构建if逻辑体布尔表达式 树

	bool makeStatementNode(Token t);//逻辑体内部语句
	bool StatementComplete();//逻辑体

	bool valuation_tasking(treeNode* Node,M_TokenType type);//赋值节点区分进程，根据声明token的类型来确定赋值任务
	bool valuation_general(treeNode* Node);//构建一般数据类型赋值节点
	bool transfor_infix_to_postfix(vector<Token> & opt_Token,vector<Token> & postfixToken);//转换表达式方式
	bool aidTransElement(vector<Token> & opt_Token,vector<Token> & postfixToken);
	nextStatus nextstatus;//用于确定表达式需要的下一个token类型
	treeNode* build_postfix_tree(vector<Token>& postfixToken);

	bool build_returnfunc(Token cToken);
	void replaceTrueFuncOnLeft(treeNode *node,treeNode*father);
	unsigned int funcIndex;
	vector<treeNode*> funcAddress;
	//	ioDIOGET,     //DioGet
	//	ioDIOSET,     //DioSet
	//	ioDIOSTATUS,  //DioStatus
	//	ioSIOGET,     //SioGet
	//	ioSIOSET,     //SioSet
	//	ioCLEARBUFF,  //ClearBuff
	//	ioSIOCTRL,    //SioCtrl
	bool reductionIODioGetFunc();
	bool reductionIODioSetFunc();
	bool reductionIODioStatusFunc();
	bool reductionIOSioGetFunc();
	bool reductionIOSioSetFunc();
	bool reductionIOClearBuffFunc();
	bool reductionIOSioCtrlFunc();

	bool valuation_string(treeNode* Node);
	bool valuation_dio(treeNode* Node);
	bool valuation_bool(treeNode* Node);

	bool valuation_posit(treeNode* Node);
	valStatus positStatus;
	bool val_posit_aidfuc(treeNode* Node);

	bool valuation_tool(treeNode* Node);
	valStatus toolStatus;
	bool val_tool_aidfuc(treeNode* Node);

	bool valuation_speed(treeNode* Node);
	void valuation_robotdata(treeNode* Node);//构建机器人数据类型赋值节点

	/***************************************************
	* 函数名:加速度
	* 作者：林旭军
	* 创建日期：2017-1-16
	* 修改日期：2017-1-16
	***************************************************/
	bool valuation_acceleration(treeNode *Node);
	/*********************end**************************/

	bool nodeMovJ(treeNode* Node);
	parameterStatus parameterMOVJ;
	bool nodeMovL(treeNode* Node);
	parameterStatus parameterMOVL;
	bool nodeMovC(treeNode* Node);
	parameterStatus parameterMOVC;

	bool nodeNoReFunc(treeNode* Node,M_TokenType type);
	//bool nodeDisablePower(treeNode* Node);
	//bool nodeEnablePower(treeNode* Node);
	bool nodeNoListFunc(treeNode* Node);
	bool nodeDelayFunc(treeNode* Node);
	bool nodeIODioLinkFunc(treeNode* Node);
	bool nodeIOSioLinkFunc(treeNode* Node);
	bool nodeBlends(treeNode* Node);

public://工具函数
	void getOneToken();
	bool match(M_TokenType type);
	bool RenameError(treeNode* Root,string str);//检测是否重名
	void reporterror(string x);

	treeNode* makeNode(Token t,NodeType type);//创建语法类型节点
	sonNode* makeSonNode(Token t);//创建语法类型 子节点

	treeNode* TraversingTreeForDeclaration(treeNode* Root,string str);//按字符串遍历（用于查找声明节点）
	bool TraversingTreeValOrNot(treeNode* Root,string str);//（用于检测整型和浮点型变量是否已经赋值）
	bool TraversingTree_othersValOrNot(treeNode* Root,string str,M_TokenType type);//（用于检测其余类型变量是否已经赋值）

	treeNode* TraversingTreeForValNode(treeNode* Root,string str,M_TokenType type);//（用于查找赋值节点）
	treeNode* TraversingTreeForMovNode(treeNode* Root,NodeType type);

protected:
	void clearTree(treeNode* root);

#ifdef _WINDOWS_VS_ ///////////////////////////////////////////////////////////编译平台

protected:
	CRobotAppApp *theApp;
	COutputWnd *p_OutputWnd;
#endif
};

#endif//define  _MYPARSE_
