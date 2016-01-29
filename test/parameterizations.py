#Expand grab parameters.
#parameters[0:3]= orientation parameters
#parameters[3]= ratio of grab position
def ExpandGrabParameters(parameters, p_1, p_2):
  a_1= parameters[0]
  a_2= parameters[1]
  a_3= parameters[2]
  ax= [math.cos(a_2)*math.cos(a_1), math.cos(a_2)*math.sin(a_1), math.sin(a_2)]
  q_g= QFromAxisAngle(ax, a_3)

  grab_ratio= parameters[3] #0.95 #Almost bottom; this should be planned(FIXME)
  p_g= grab_ratio*Vec(p_1) + (1.0-grab_ratio)*Vec(p_2)
  return list(p_g)+list(q_g)

#Expand grab parameters.
#parameters[0:3]= orientation parameters
#parameters[3]= ratio of grab position
def ExpandGrabParameters(parameters, p_1, p_2):
  a_1= parameters[0]
  a_2= parameters[1]
  a_3= parameters[2]
  ez= [math.sin(a_2)*math.cos(a_1), math.sin(a_2)*math.sin(a_1), math.cos(a_2)]
  ex= [math.cos(a_3), math.sin(a_3), 0.0]
  ex= Orthogonalize(ex, base=ez, original_norm=False)
  ey= np.cross(ez,ex)
  R= np.array([[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]])
  R[:,0]= ex
  R[:,1]= ey
  R[:,2]= ez
  q_g= RotToQ(R)

  grab_ratio= parameters[3] #0.95 #Almost bottom; this should be planned(FIXME)
  p_g= grab_ratio*Vec(p_1) + (1.0-grab_ratio)*Vec(p_2)
  return list(p_g)+list(q_g)


