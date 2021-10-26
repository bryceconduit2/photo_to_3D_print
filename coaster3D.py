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
def cylinder(height,radius):
  no_points = 500
  cyl_x_u = np.zeros((no_points))
  cyl_y_u = np.zeros((no_points))
  cyl_z_u = np.zeros((no_points))
  cyl_x_l = np.zeros((no_points))
  cyl_y_l = np.zeros((no_points))
  cyl_z_l = np.zeros((no_points))
  theta = np.linspace(0.,2.*math.pi,no_points)
  for i,t in enumerate(theta):
    cyl_x_u[i] = radius* math.cos(t)
    cyl_y_u[i] = radius*math.sin(t)
    cyl_x_l[i] = (radius+0.5)*math.cos(t)
    cyl_y_l[i] = (radius+0.5)*math.sin(t)
  cyl_z_u[:] = height
  cyl_z_l[:] = 0.
  return(cyl_x_u,cyl_y_u,cyl_z_u,cyl_x_l,cyl_y_l,cyl_z_l)

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
  x = np.zeros((maxx*maxy))
  y = np.zeros((maxx*maxy))
  z = np.zeros((maxx*maxy))
  for i in range(0,maxx-2):
    for j in range(0,maxy-2):
      x[i*maxy+j] = (i - maxx*0.5+shiftx)
      y[i*maxy+j] = (j- maxy*0.5+shifty)
      z[i*maxy+j] = (height - pixels[i,j]/heightdivide)
  return(x,y,z)
  
#joins it all together into point cloud
def append_to_point_cloud(x_stl,y_stl,z_stl,x,y,z):
  x_stl = np.append(x_stl,x)
  y_stl = np.append(y_stl,y)
  z_stl = np.append(z_stl,z)
  return(x_stl,y_stl,z_stl)


if __name__ == "__main__":
  im = Image.open('example.jpg') # Can be many different formats.
  text = Image.open('exampletext.jpg') # Can be many different formats.
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
  cyl_x_u,cyl_y_u,cyl_z_u,cyl_x_l,cyl_y_l,cyl_z_l = cylinder(height,radius)

  #convolution filter for smoothing
  conv_filter = np.array([[1,1,1],[1,1,1],[1,1,1]])
  pixels = convolve2d(pixels, conv_filter)
  pixelstext = convolve2d(pixelstext, conv_filter)
  print(len(pixels))
  
  #position picture
  x,y,z=position(maxx,maxy, height,pixels,300,10,0)

  x_stl = np.empty((0))
  y_stl = np.empty((0))
  z_stl = np.empty((0))
  #combine 3d piccy into big point cloud
  x_stl,y_stl,z_stl = append_to_point_cloud(x_stl,y_stl,z_stl,x,y,z)
  x_stl,y_stl,z_stl = append_to_point_cloud(x_stl,y_stl,z_stl,cyl_x_u,cyl_y_u,cyl_z_u)
  x_stl,y_stl,z_stl = append_to_point_cloud(x_stl,y_stl,z_stl,cyl_x_l,cyl_y_l,cyl_z_l)
  
  #position text
  textx,texty,textz=position(maxxtext,maxytext, height,pixelstext,700,-60,0)

  #combine 3d text into big point cloud
  x_stl,y_stl,z_stl = append_to_point_cloud(x_stl,y_stl,z_stl,textx,texty,textz)

  #define 2D points, as input data for the Delaunay triangulation of U
  points2D=np.vstack([x_stl,y_stl]).T
  tri = Delaunay(points2D)#triangulate the rectangle U
  #define 3D points
  points3D=np.vstack([x_stl,y_stl,z_stl]).T
  # Create the mesh
  stl_mesh = mesh.Mesh(np.zeros(len(tri.simplices), dtype=mesh.Mesh.dtype))
  for i, f in enumerate(tri.simplices):
    for j in range(3):
      stl_mesh.vectors[i][j] = points3D[f[j],:]

  #save file
  stl_mesh.save('output.stl')


