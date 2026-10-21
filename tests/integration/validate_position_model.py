#!/usr/bin/env python3
"""Compile the actual get_packages_path function with a read-only ament stub."""
import argparse
from pathlib import Path
import shutil
import subprocess
import tempfile

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--source',type=Path,required=True)
args=parser.parse_args()
source=(args.source/'dual_arm_control/src/controller/ik_solver_wrapper.cpp').read_text()
a=source.index('std::string get_packages_path(')
b=source.index('IKSolverWrapper::IKSolverWrapper()',a)
function=source[a:b]
preamble=r'''
#include <cassert>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
static unsigned mode=0;static std::string package_path;
namespace ament_index_cpp {
std::string get_package_share_directory(const std::string&){
 if(mode==1)throw std::runtime_error("package absent");
 return mode==2 ? package_path+"/missing" : package_path;
}}
'''
test=r'''
int main(int argc,char** argv){
 if(argc!=2)return 2;package_path=argv[1];
 assert(get_packages_path("dual_arm","urdf")==package_path+"/urdf");
 for(mode=1;mode<=2;++mode){
  bool caught=false;
  try{(void)get_packages_path("dual_arm","urdf");}
  catch(const std::runtime_error& e){caught=std::string(e.what()).find("dual_arm/urdf")!=std::string::npos;}
  assert(caught);
 }
 std::cout<<"3 actual model-path branches passed\n";
}
'''
with tempfile.TemporaryDirectory(prefix='model-path-review-') as tmp:
    root=Path(tmp);(root/'urdf').mkdir();cpp=root/'check.cpp';cpp.write_text(preamble+function+test)
    compiler=shutil.which('c++')
    if not compiler:raise RuntimeError('C++ compiler unavailable')
    result=subprocess.run([compiler,'-std=c++17','-Wall','-Wextra','-Werror',str(cpp),'-o',str(root/'check')],capture_output=True,text=True)
    if result.returncode:raise RuntimeError(result.stderr)
    result=subprocess.run([str(root/'check'),str(root)],check=True,capture_output=True,text=True)
    print(result.stdout.strip())
