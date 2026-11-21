"""Build the small pinned compatibility guard outside source; no package installs."""
from pathlib import Path
import argparse,json,subprocess,sys
from tools.flight.gazebo_guard import SOURCE,PLUGIN,SIM,sha,ROOT
def main():
    p=argparse.ArgumentParser();p.add_argument('output',type=Path);a=p.parse_args();out=a.output.resolve()
    if sys.platform!='linux':raise ValueError('Build against the installed Gazebo Sim 8 Linux runtime')
    if out==ROOT or ROOT in out.parents:raise ValueError('Build must remain outside source')
    plugin_sha,sim_sha=sha(PLUGIN),sha(SIM)
    out.mkdir(parents=True,exist_ok=False);binary=out/'libgz_sensor_stop_guard.so'
    command=['g++','-shared','-fPIC','-O2','-o',str(binary),str(SOURCE),'-ldl','-pthread']
    with (out/'build.log').open('x') as log:subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True)
    meta=dict(schema=1,source_sha256=sha(SOURCE),binary_sha256=sha(binary),sensors_plugin_sha256=plugin_sha,sim_sha256=sim_sha,compiler=subprocess.check_output(['g++','--version'],text=True).splitlines()[0],command=command,diagnostic_injection=False)
    Path(str(binary)+'.json').write_text(json.dumps(meta,indent=2)+'\n');print(json.dumps(meta))
if __name__=='__main__':main()
