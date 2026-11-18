from test_native_static_events import compile_run


def test_mission_budget_units_reserves_and_no_per_target_reset(tmp_path):
    compile_run(tmp_path,'''#include "static_mission_budget.hpp"
#include <cassert>
using namespace racer_task;
int main(){StaticMissionBudget b;assert(!b.allows(0.,0.));
 b={100.,10.,2.,5000.,500.,50.,"synthetic-power-bound"};
 assert(b.configured()&&!b.allows(0.,1.));assert(b.start(10.));assert(!b.start(50.));
 assert(b.allows(10.,10.));assert(!b.allows(9.,0.));
 assert(b.allows(80.,18.)); // 70 elapsed +18 trajectory +2 observe +10 reserve =100s
 assert(!b.allows(80.,18.1));assert(!b.allows(100.,0.));
 b.initial_energy_j=3000.;
 assert(b.allows(57.,1.)); // 47 elapsed +1 trajectory +2 observe at 50W plus 500J
 assert(!b.allows(60.,1.)); // 50+1+2 seconds *50W +500J >3000J
 assert(b.allows(10.,10.));b.model_id.clear();assert(!b.allows(10.,0.));
}
''')
