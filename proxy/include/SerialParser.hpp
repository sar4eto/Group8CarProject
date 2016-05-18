#include <opendavinci/odcore/io/StringListener.h>
#include <vector>

class SerialParser : public odcore::io::StringListener {
	
	public:
		virtual std::vector<double> getIR();
    	virtual std::vector<double> getUS();
    	virtual double getDist();
    	virtual bool irHasNewVals();
    	virtual bool usHasNewVals();

    virtual void nextString(const std::string &s);
};