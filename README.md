"# photo_to_3D_print" 
This script (coaster3d.py) takes in "example.jpg" and "exampletitle.jpg" and creates a *.stl coaster suitable for 3D printing by applying a pixel height formula to the two input image files. It applies a convolution to create a smooth surface. Uses delaunay triangulation to create the mesh. I use Repetier to print this which can handle the limitations of the delaunay triangulation.

![Alt text](/output/outputstl.jpg?raw=true "Title")

Prerequisities: \
Open anaconda prompt. Anaconda/miniconda available from https://www.anaconda.com/.  \

Installation:
1) Create conda/miniconda environment and activate:\
conda create --name 3d \
conda activate 3d 
2) Install python\
conda install -c anaconda python=3.8
3) Clone/download github repositrary into desired folder.\
git clone https://github.com/bryceconduit2/photo_to_3D_print.git
4) cd into directory\
cd photo_to_3D_print
5) Install required python libraries:\
pip install -r requirements.txt. If problems are encountered here with installation of packages: - pip install pipreqs; pipreqs --force and repeat. You may need to install numpy-stl via pip install numpy-stl
6) Enter src directory\
cd src
7) Run python script\
python coaster3D.py. If "from stl import mesh" error encountered here do pip install numpy-stl.\
