from PIL import Image
import PIL.ImageOps  
import numpy as np
from scipy.spatial import Delaunay
from stl import mesh
import math

#convolution for smoothing
def convolve2d(a, conv_filter):
  submatrices = np.array([
         [a[:-2,:-2], a[:-2,1:-1], a[:-2,2:]],
         [a[1:-1,:-2], a[1:-1,1:-1], a[1:-1,2:]],
         [a[2:,:-2], a[2:,1:-1], a[2:,2:]]])
  multiplied_subs = np.einsum('ij,ijkl->ijkl',conv_filter,submatrices)
  return np.sum(np.sum(multiplied_subs, axis = -3), axis = -3)

#creates cylinder coaster shape
def circle(height,radius):
  no_points = 500
  circle_ = np.zeros((no_points,3))
  theta = np.linspace(0.,2.*math.pi,no_points)
  for i,t in enumerate(theta):
    circle_[i,0] = radius* math.cos(t)
    circle_[i,1] = radius*math.sin(t)
  circle_[:,2] = height
  return(circle_)

#converts from pixel colour to point height using formula
def pixel_height(pix,im):
  maxx = im.size[0]
  maxy = im.size[1]
  pixels = np.zeros((maxx,maxy))
  for i in range(maxx):
    for j in range(maxy):
      pixels[i,j] = 0.2989 * pix[i,j][0] + 0.5870 * pix[i,j][1] + 0.1140 * pix[i,j][2]
  return(pixels,maxx,maxy)

#shifts stuff around on coaster surface
def position(maxx,maxy, height,pixels,heightdivide,shiftx,shifty):
  position_ = np.zeros((maxx*maxy,3))
  for i in range(0,maxx-2):
    for j in range(0,maxy-2):
      position_[i*maxy+j,0] = (i - maxx*0.5+shiftx)
      position_[i*maxy+j,1] = (j- maxy*0.5+shifty)
      position_[i*maxy+j,2] = (height - pixels[i,j]/heightdivide)
  return(position_)
  
#joins it all together into point cloud
def append_to_point_cloud(points3D,points3D_):
  points3D = np.append(points3D,points3D_,axis=0)
  return(points3D)


if __name__ == "__main__":
  im = Image.open('../input/example.jpg') # Can be many different formats.
  text = Image.open('../input/exampletext.jpg') # Can be many different formats.
  #flip image to get black background
  im = PIL.ImageOps.invert(im)
  pix = im.load()
  
  
  textpix = text.load()
  print(im.size)  # Get the width and hight of the image for iterating over
  print(text.size)
  pixels,maxx,maxy = pixel_height(pix,im)
  pixelstext,maxxtext,maxytext = pixel_height(textpix,text)

  #height and radius of coaster
  height = 9.
  radius = 105.

  #create cylinder for plot
  cyl_u = circle(height,radius)
  cyl_l = circle(0,radius+0.5)

  #convolution filter for smoothing
  conv_filter = np.array([[1,1,1],[1,1,1],[1,1,1]])
  pixels = convolve2d(pixels, conv_filter)
  pixelstext = convolve2d(pixelstext, conv_filter)
  print(len(pixels))
  
  #position picture
  new_position=position(maxx,maxy, height,pixels,300,10,0)

  points3D = np.empty((0,3))

  #combine 3d piccy into big point cloud
  points3D = append_to_point_cloud(points3D,new_position)
  points3D = append_to_point_cloud(points3D,cyl_u)
  points3D = append_to_point_cloud(points3D,cyl_l)
  
  #position text
  text_position=position(maxxtext,maxytext, height,pixelstext,700,-60,0)

  #combine 3d text into big point cloud
  points3D = append_to_point_cloud(points3D,text_position)

  #define 2D points, as input data for the Delaunay triangulation of U
  points2D=points3D[:,0:2]
  tri = Delaunay(points2D)#triangulate the rectangle U

  # Create the mesh
  stl_mesh = mesh.Mesh(np.zeros(len(tri.simplices), dtype=mesh.Mesh.dtype))
  for i, f in enumerate(tri.simplices):
    for j in range(3):
      stl_mesh.vectors[i][j] = points3D[f[j],:]

  #save file
  stl_mesh.save('../output/output.stl')


