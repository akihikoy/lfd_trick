#!/usr/bin/python
from core_tool import *
def Help():
  return '''Generating object elements.
  Usage:
    objgen 'pedge', 'cir', Z_POS, RADIUS [, N_DIV [, TH_START [, TH_END [, OFFSET [, MAT]]]]
      Generate a set of pouring edge points of circle type.
      Z_POS: Z-position.
      RADIUS: Radius of the circle.
      N_DIV: Number of dividing points (default: 20).
      TH_START: Start angle (default: 0 [deg]).
      TH_END: End angle (default: 360 [deg]).
      OFFSET: Offset of x,y (default: [0.0,0.0]).
      MAT: Matrix applied to x,y (default: [[1.0,0.0],[0.0,1.0]]).
      Note: p= [MAT*(px,py)+OFFSET, Z_POS] where (px,py) is a point on the circle of RADIUS.
    objgen 'cone', Z_START, Z_END, DIAM_START, DIAM_END, N_DIV [, PRIM_TYPE [, THICK]]
      Generate tubes or cylinders to model a cone.
      Z_START, Z_END: bottom and top z-heights.
      DIAM_START, DIAM_END: bottom and top diameters.
      PRIM_TYPE: primitive type (rtpkTube or rtpkCylinder, default:rtpkTube).
      THICK: thickness of a tube (default: 0.002).
  '''
def Run(t,*args):
  assert(len(args)>0)
  command= args[0]
  args= args[1:]
  if command=='pedge':
    pedge= []
    if args[0]=='cir':
      z_pos= args[1]
      radius= args[2]
      n_div= args[3] if len(args)>3 else 20
      th_start= DegToRad(float(args[4]) if len(args)>4 else 0.0)
      th_end= DegToRad(float(args[5]) if len(args)>5 else 360.0)
      offset= args[6] if len(args)>6 else [0.0,0.0]
      mat= args[7] if len(args)>7 else [[1.0,0.0],[0.0,1.0]]
      offset= np.array(offset)
      mat= np.array(mat)
      for i in range(n_div):
        angle= th_start + (th_end-th_start)*float(i)/float(n_div)
        p= [radius*math.cos(angle),radius*math.sin(angle)]
        p= np.dot(mat,p)+offset
        p= p.tolist()+[z_pos]
        pedge.append(p)
    for p in pedge:
      print '- %r' % p
  elif command=='cone':
    z_start= args[0]
    z_end= args[1]
    diam_start= args[2]
    diam_end= args[3]
    n_div= args[4]
    prim_type= args[5] if len(args)>5 else 'rtpkTube'
    thick= args[6] if len(args)>6 else 0.002
    if n_div<2:
      print 'N_DIV should be >=2.'
      return
    z= z_start
    for i in range(n_div):
      rad= 0.5*(float(i)/float(n_div-1)*(diam_end-diam_start) + diam_start)
      height= (z_end-z_start)/float(n_div-1)
      if i in (0,n_div-1):  height*= 0.5
      if prim_type=='rtpkTube':
        prim='''- kind: {prim_type}
  param: [{rad_out}, {rad_in}, {height}, 0.0,0.0]
  pose: [0.0,0.0,{z}, 0.0,0.0,0.0,1.0]'''.format(prim_type=prim_type, rad_out=rad, rad_in=rad-thick, height=height, z=z+0.5*height)
      elif prim_type=='rtpkCylinder':
        prim='''- kind: {prim_type}
  param: [{rad}, {height}]
  pose: [0.0,0.0,{z}, 0.0,0.0,0.0,1.0]'''.format(prim_type=prim_type, rad=rad, height=height, z=z+0.5*height)
      print prim
      z+= height
