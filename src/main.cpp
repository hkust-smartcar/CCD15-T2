#include "app.h"
#include <libsc/system.h>
#include <libutil/misc.h>
#include <libsc/led.h>
#include <functional>

using namespace libsc;
using namespace libsc::kl26;
using namespace libbase::kl26;
using namespace libutil;

Led::Config GetLedConfig2(int id){
	libsc::Led::Config ledconfig;
	ledconfig.id = id;
//	ledconfig.is_active_low = false;
	return ledconfig;
}

int main(){
	System::Init();

	App m_app;

	return 0;
}
