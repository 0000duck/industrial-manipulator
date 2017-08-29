/*
 * Config.h
 *
 *  Created on: Aug 29, 2017
 *      Author: a1994846931931
 */

#ifndef CONFIG_H_
#define CONFIG_H_

# include "../math/Q.h"

namespace robot {
namespace model {



class Config {
public:
	static const int righty=2, lefty=1, ssame=0, sfree=-1;
	static const int epositive=2, enegative=1, esame=0, efree=-1;
	static const int wpositive=2, wnegative=1, wsame=0, wfree=-1;
public:
	Config();
	Config(int shoulder, int elbow, int wrist);
	void setShoulder(int);
	void setElbow(int);
	void setWrist(int);
	int getShoulder() const;
	int getElbow() const;
	int getWrist() const;
	virtual ~Config();
private:
	int _shoulder;
	int _elbow;
	int _wrist;
};

} /* namespace model */
} /* namespace robot */

#endif /* CONFIG_H_ */
