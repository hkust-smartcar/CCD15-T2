#include <libbase/k60/mcg.h>
#include <libsc/k60/system.h>
#include "car.h"
//using namespace libsc::kl26;
//using namespace libbase::kl26;


using namespace libsc::k60;
using namespace libbase::k60;

namespace libbase
{
namespace k60
{

Mcg::Config Mcg::GetMcgConfig()
{
	Mcg::Config config;
	config.external_oscillator_khz = 50000;
	config.core_clock_khz = 150000;
	return config;
}

}
}

int main(){
	System::Init();

	Car m_car;
	m_car.Run();

	while(true);

	return 0;
}
