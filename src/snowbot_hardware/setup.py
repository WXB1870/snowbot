from pathlib import Path
from setuptools import setup, find_packages

NAME = 'snowbot_hardware'
assets = [p for folder in ("config", "launch", "urdf", "meshes", "sources") for p in Path(folder).rglob("*") if p.is_file() and "__pycache__" not in p.parts]
assets += [p for p in Path(".").glob("*NOTICE") if p.is_file()]
data = [("share/"+NAME+"/"+str(parent), [str(p) for p in assets if p.parent == parent]) for parent in sorted({p.parent for p in assets})]
setup(name=NAME, version="0.1.0", packages=find_packages(),
      data_files=[("share/ament_index/resource_index/packages", ["resource/"+NAME]), ("share/"+NAME, ["package.xml"]), *data],
      install_requires=["setuptools"], zip_safe=False,
      entry_points={"console_scripts": ['snowbot_adapter = snowbot_hardware.adapter:main', 'snowbot_position_sim = snowbot_hardware.position_sim:main']})
