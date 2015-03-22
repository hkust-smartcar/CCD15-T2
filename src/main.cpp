#include "app.h"
#include <libsc/system.h>
#include <libbase/k60/mcg.h>
#include <libutil/misc.h>
#include <functional>

using namespace libsc;
using namespace libsc::k60;
using namespace libbase::k60;
using namespace libutil;


int main(){
	System::Init();

	App m_app;

	return 0;
}
