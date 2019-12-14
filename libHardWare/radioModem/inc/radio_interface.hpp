#include "ifile_descriptor_owner.h"
#include "serial_port.hpp"
#include "serial_port_exception.h"
#include "serial_port_util.h"
#include "Commands.hpp"
#include <mutex>
#include "base_thread.h"

using namespace std;
using namespace mrobot;

class RadioInterface : public BaseThread
{

    private:
        string message;
        Commands *commands;
        float kp, ki, kd;
	mutex mut;
    public:
        void run();
        void getPIDParam(float &kp, float &ki, float &kd);

};


