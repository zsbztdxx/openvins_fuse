#include "UpdaterGNSS.h"

using namespace ov_msckf;

UpdaterGNSS::UpdaterGNSS(NoiseManager &noises, std::shared_ptr<Propagator> prop): _noises(noises), _prop(prop)
{
 
    //todo
}