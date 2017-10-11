/*
 * RobotXMLParser.cpp
 *
 *  Created on: Oct 10, 2017
 *      Author: a1994846931931
 */

#include "RobotXMLParser.h"
# include <iostream>
# include <regex>
# include <fstream>

namespace robot {
namespace parse {

RobotXMLParser::RobotXMLParser() {

}


SerialLink::ptr RobotXMLParser::parse(const char* filename)
{
	return parse(string(filename));
}

SerialLink::ptr RobotXMLParser::parse(const std::string filename)
{
	SerialLink::ptr robot(new SerialLink());
	std::vector<Link::ptr> linkList;
	std::ifstream fin(filename, std::ifstream::in);
	string temp;
	string fstr;
	while(fin >> temp)
	{
		fstr.append(temp);
	}
	std::smatch tempMatch;
	std::regex tagRobot("(<robot>)(.*)(</robot>)", std::regex::icase);
	if (regex_search(fstr, tempMatch, tagRobot))
	{
		string body = tempMatch.str(2);
		std::regex tagName("(<name>)(.*?)(</name>)(.*)", std::regex::icase);
		if (regex_match(body, tagName))
		{
			cout << "Name:\t" << std::regex_replace(body, tagName, "$2") << endl;
		}
		else
		{
			cout << "Name:\t" << endl;
		}
		cout << '\n';
		std::regex tagJoint("(<joint>)(.*?)(</joint>)", std::regex::icase);
		std::regex tagAlpha("(<alpha>)(.*?)(</alpha>)", std::regex::icase);
		std::regex tagA("(<a>)(.*?)(</a>)", std::regex::icase);
		std::regex tagD("(<d>)(.*?)(</d>)", std::regex::icase);
		std::regex tagTheta("(<theta>)(.*?)(</theta>)", std::regex::icase);
		std::regex tagLmin("(<min>)(.*?)(</min>)", std::regex::icase);
		std::regex tagLmax("(<max>)(.*?)(</max>)", std::regex::icase);
		for (std::sregex_iterator it(body.cbegin(), body.cend(), tagJoint), end; it!=end; it++)
		{
			string alpha;
			string a;
			string d;
			string theta;
			string lmin;
			string lmax;
			string name;
			string jointStr = it->str();
			cout << "Joint: ";
			if (regex_search(jointStr, tempMatch, tagName))
			{
				name = tempMatch.str(2);
				cout << name <<"\n";
			}
			else
			{
				cout << "\tname:\n";
			}
			if (regex_search(jointStr, tempMatch, tagAlpha))
			{
				alpha = tempMatch.str(2);
				cout << " -alpha:" << alpha <<"\n";
			}
			else
			{
				throw( string( "Invalid model file! Could not find 'alpha' tag in joint"));
			}
			if (regex_search(jointStr, tempMatch, tagA))
			{
				a = tempMatch.str(2);
				cout << " -a:\t" << a <<"\n";
			}
			else
			{
				throw( string( "Invalid model file! Could not find 'a' tag in joint"));
			}
			if (regex_search(jointStr, tempMatch, tagD))
			{
				d = tempMatch.str(2);
				cout << " -d:\t" << d <<"\n";
			}
			else
			{
				throw( string( "Invalid model file! Could not find 'd' tag in joint"));
			}
			if (regex_search(jointStr, tempMatch, tagTheta))
			{
				theta = tempMatch.str(2);
				cout << " -theta:" << theta <<"\n";
			}
			else
			{
				throw( string( "Invalid model file! Could not find 'theta' tag in joint"));
			}
			if (regex_search(jointStr, tempMatch, tagLmin))
			{
				lmin = tempMatch.str(2);
				cout << " -min:" << lmin <<"\n";
			}
			else
			{
				throw( string( "Invalid model file! Could not find 'min' tag in joint"));
			}
			if (regex_search(jointStr, tempMatch, tagLmax))
			{
				lmax = tempMatch.str(2);
				cout << " -max:" << lmax <<"\n";
			}
			else
			{
				throw( string( "Invalid model file! Could not find 'max' tag in joint"));
			}
			double dalpha;
			double da;
			double dd;
			double dtheta;
			double dlmin;
			double dlmax;
			try{
				dalpha = std::stod(alpha)/180.0*M_PI;
				da = std::stod(a);
				dd = std::stod(d);
				dtheta = std::stod(theta)/180.0*M_PI;
				dlmin = std::stod(lmin)/180.0*M_PI;
				dlmax = std::stod(lmax)/180.0*M_PI;
			}
			catch(const char* msg)
			{
				cout << "error: " << msg << '\n';
				throw( string("XML文件格式错误!") );
			}
			linkList.push_back(Link::ptr(new Link(dalpha, da, dd, dtheta, dlmin, dlmax, name)));
			cout << '\n';
		}
	}
	else
	{
		throw(string("Invalid model file! Could not find 'robot' tag\n"));
	}
	if (linkList.empty())
	{
		cout << "警告<RobotXMLParser>: 关节的个数为0!\n";
	}
	for (int i=0; i<(int)linkList.size(); i++)
	{
		robot->append(linkList[i]);
	}
	return robot;
}

RobotXMLParser::~RobotXMLParser() {
}

} /* namespace parse */
} /* namespace robot */
