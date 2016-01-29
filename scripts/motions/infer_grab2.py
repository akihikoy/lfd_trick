#!/usr/bin/python
from core_tool import *
def Help():
  return '''Infer grab pose l_x_grab, its width g_width, and pre grab pose l_x_grab0.
  Ver.2: Instead of grab_primitives, we use shape_primitives,
    and use CompOptInference as an optimizer for composite parameters.
  Usage: infer_grab SITUATION, ATTR_KEYS [, INFERRED_KEYS [, VERBOSE]]
    SITUATION: dictionary of situation.  e.g. {'p':[0,0,0]}
    ATTR_KEYS: list of attributes keys. e.g. ['b1']
    INFERRED_KEYS: list of inferred keys is stored
    VERBOSE: if True, inferred information is printed (default:True)'''

#Evaluate a grab pose of a cylinder model for pouring
#l_x_g: grab point and orientation in the source container frame
#l_x_g0: pre-grab point and orientation in the source container frame
#g_width: grab width
#l_p_1, l_p_2: cylinder (bottom and top points) of source container
#width: width of the cylinder
def EvalGrabPose(dsit, l_x_g, l_x_g0, g_width, idx, verbose=False):
  if g_width/dsit.grab_margin>0.08: return None  #TODO:provide by attributes
  if Dist(dsit.l_p_1[idx],dsit.l_p_2[idx])<0.02: return None  #TODO:provide by attributes
  score= 1.0

  p_g,R_g= XToPosRot(Transform(dsit.x_b, l_x_g))
  p_1= Transform(dsit.x_b, dsit.l_p_1[idx])
  p_2= Transform(dsit.x_b, dsit.l_p_2[idx])
  e_x= R_g[:,0]
  e_y= R_g[:,1]
  e_z= R_g[:,2]

  p_ec= Transform(dsit.x_b,dsit.edge_b_center)

  #TEST
  ##Requirement from pouring:
  #a1= GetAngle(e_x, (Vec(dsit.x_c)-Vec(dsit.x_b))[0:3])
  #if verbose: print 'a1=',RadToDeg(a1)
  #if not IsIn(a1,DegToRad([70,110])):  return None
  #score-= abs(a1-DegToRad(90))/math.pi

  #Score from pouring:
  ax= Normalize(Vec(p_2)-Vec(p_1))
  hc= (p_ec-Vec(p_1)) - np.dot(p_ec-Vec(p_1), ax)*ax
  if verbose: print 'Norm(hc)=',Norm(hc)
  if Norm(hc) > 0.2*g_width:
    a1= GetAngle(-e_y, hc)
    if verbose: print 'a1=',RadToDeg(a1)
    score-= 2.0*abs(a1)/math.pi

  #TEST
  ##Requirement from left hand grab pose:
  #a2= math.atan2(e_x[1],e_x[0])
  #if verbose: print 'a2=',RadToDeg(a2)
  ##if not IsIn2(a2,(DegToRad([170,180]),DegToRad([-180,30]))):  return None
  #if not IsIn(a2,DegToRad([-100,100])):  return None
  #score-= abs(a2)/math.pi

  #Score from current hand pose and grab pose (considering only xy plane):
  p_w, R_w= XToPosRot(dsit.x_w)
  e_x_w= R_w[:,0]
  #a2= abs(GetAngle(e_x, e_x_w))  #[0,pi]
  a2= abs(GetAngle([e_x[0],e_x[1],0.0], [e_x_w[0],e_x_w[1],0.0]))  #[0,pi]
  if verbose: print 'a2=',RadToDeg(a2)
  score-= 0.5*abs(a2)/math.pi

  #Requirement and score from cylinder grab pose:
  a3= GetAngle(e_z, Vec(p_2)-Vec(p_1))
  if verbose: print 'a3=',RadToDeg(a3)
  if not IsIn(a3,DegToRad([-45,45])):  return None
  score-= 3.0*abs(a3)/math.pi  #better to be zero

  #TEST
  ##Requirement from hand orientation:
  #a4= math.atan2(e_x[2],la.norm([e_x[0],e_x[1]]))
  #if verbose: print 'a4=',RadToDeg(a4)
  ##if p_g[2]-dsit.x_b[2] <= 0.05:  # Height of grab point is less than 5cm
    ##if not IsIn(a4, DegToRad([-70,-20])):  return None
  ##else:
    ##if not IsIn(a4, DegToRad([-70,0])):  return None
  #if not IsIn(a4, DegToRad([-70,-20])):  return None
  #score-= abs(a4)/(0.5*math.pi)

  #Requirement for pouring margin:
  #FIXME: should use l_x_pour_e (i.e. do the pouring parameter estimation first!)
  a5= p_2[2]-p_g[2]
  if verbose: print 'a5=',a5
  if a5 < 0.03:  # Top margin for grab is less than 3cm
    return None
  if a5 < 0.05:  # Top margin for grab is less than 5cm
    score-= (0.05-a5)/0.05

  #Check the validity of grab pose-0
  x_g0= Transform(dsit.x_b,l_x_g0)
  x_w_trg= TransformRightInv(x_g0,dsit.lw_xe)  #Target of wrist x
  vg,res2,res3= dsit.t.ExecuteMotion('scene','isvalidx',dsit.arm,x_w_trg,[],dsit.q_curr)
  #if verbose:  print 'res2.error_code.val=',res2.error_code.val if res2 else None
  #if verbose:  print 'res3.error_code.val=',res3.error_code.val if res3 else None
  if verbose:  print 'res2=',res2
  if verbose:  print 'res3=',res3
  if verbose and res3:
    #print 'res3:\n',res3
    VisualizeContacts(res3.contacts,dt=rospy.Duration(0.5))
  if not vg:  return None

  #Check the validity of grab pose
  x_g= Transform(dsit.x_b,l_x_g)
  x_w_trg= TransformRightInv(x_g,dsit.lw_xe)  #Target of wrist x
  ignore= [dsit.bottle]  #Ignoring collision with bottle
  vg,res2,res3= dsit.t.ExecuteMotion('scene','isvalidx',dsit.arm,x_w_trg,ignore,dsit.q_curr)
  #if verbose:  print 'res2.error_code.val=',res2.error_code.val if res2 else None
  #if verbose:  print 'res3.error_code.val=',res3.error_code.val if res3 else None
  if verbose:  print 'res2=',res2
  if verbose:  print 'res3=',res3
  if verbose and res3:
    #print 'res3:\n',res3
    VisualizeContacts(res3.contacts,dt=rospy.Duration(0.5))
  if not vg:  return None

  return score

#Expand grab parameters.
#parameters[0,1]= orientation parameters, in [-pi,pi]x[-0.5pi,0.5pi]
#parameters[2]= ratio of grab position (ratio*l_p_1+(1-ratio)*l_p_2)
#l_p_1, l_p_2: cylinder (bottom and top points) of source container
#width: cylinder width
def ExpandGrabParameters(parameters, dsit):
  idx= parameters[0]
  if idx>=len(dsit.shapes):  return None,None,None
  if dsit.shapes[idx]=='cyl':
    if not CompHasStruct(parameters[1], (float,float,float)):  return None,None,None
    cylgparameters= parameters[1]
    width= dsit.width[idx]
  elif dsit.shapes[idx]=='cub':
    if not CompHasStruct(parameters[1], (int,(float,float))):  return None,None,None
    cylgparameters= [dsit.idx2angle[parameters[1][0]]]+list(parameters[1][1])
    width= dsit.width[idx][parameters[1][0]]
  #print 'cylgparameters=',cylgparameters
  a_1= cylgparameters[0]
  a_2= cylgparameters[1]
  #ez= Normalize(Vec(dsit.l_p_2[idx])-Vec(dsit.l_p_1[idx]))
  #ex= GetOrthogonalAxisOf(ez, preferable=[1.0,0.0,0.0], fault=[0.0,0.0,-1.0])
  #ex= np.dot(RFromAxisAngle(ez,a_1),ex)
  #ey= np.cross(ez,ex)
  #R= Eye()
  #R[:,0]= ex
  #R[:,1]= ey
  #R[:,2]= ez
  ez= dsit.R[idx][:,2]
  R= np.dot(RFromAxisAngle(dsit.R[idx][:,2],a_1), dsit.R[idx])
  ex= R[:,0]  #For future use
  ey= R[:,1]
  #ez= R[:,2]
  l_q_g= RotToQ(np.dot(RFromAxisAngle(ey,a_2), R))

  grab_ratio= cylgparameters[2]
  l_p_g= grab_ratio*Vec(dsit.l_p_1[idx]) + (1.0-grab_ratio)*Vec(dsit.l_p_2[idx])
  l_x_g= list(l_p_g)+list(l_q_g)

  #Estimating pre-grab pose l_x_g0
  #Method A
  #l.x_grab0= copy.deepcopy(l.x_grab)
  #l.x_grab0[0]-= dsit.t.GetAttr(obj,'g_width')  #TODO: generalize
  #l.x_grab0[3:7]= dsit.t.robot.FK(arm=handid)[3:7]
  #Method B
  #p_g,R_g= XToPosRot(l.x_grab)
  #p_g0= Vec(p_g) - np.dot(R_g[:,0], 1.5*dsit.t.GetAttr(obj,'g_width'))  #TODO: generalize
  #l.x_grab0= PosRotToX(p_g0,R_g)

  g_width= dsit.grab_margin*width
  l_p_g0= Vec(l_p_g) - np.dot(ex, 1.5*width)  #FIXME: magic number
  l_x_g0= list(l_p_g0)+list(l_q_g)

  return l_x_g, l_x_g0, g_width

#Distance of two parameters
def DistOfGrabParams(p1,p2):
  return CompDist(p1,p2,1.0e6)  #FIXME(consider shape variations of shape_primitives)

#Distance to modify the score function from examples
def DistOfGrabPoseExs(parameters, dsit, db_situation):
  if db_situation['infer_info']['type']!='ver2':  return None
  dists= []
  dists.append(DistOfGrabParams(parameters, db_situation['infer_info']['param']))
  d_x_b= DiffX(dsit.x_b, db_situation['source_attr']['x'])
  dists.append(Norm(d_x_b[:3]))
  dists.append(0.2*Norm(d_x_b[3:]))
  d_x_c= DiffX(dsit.x_c, db_situation['receiver_attr']['x'])
  dists.append(Norm(d_x_c[:3]))
  dists.append(0.2*Norm(d_x_c[3:]))
  #print dists
  dist= max(dists)
  return dist

def EvalGrabPoseForFMax(parameters, dsit, f_none=100):
  l_x_g, l_x_g0, g_width= ExpandGrabParameters(parameters, dsit)
  if l_x_g==None:  return f_none

  score= EvalGrabPose(dsit, l_x_g, l_x_g0, g_width, parameters[0])
  score= ModifyScoreFromExamples(
      score= score,
      bad_exs= dsit.t.GetAttrOr([], 'memory','grab','bad'),
      good_exs= dsit.t.GetAttrOr([], 'memory','grab','good'),
      dist_func= lambda db_situation, inferred_data, assessment: DistOfGrabPoseExs(parameters, dsit, db_situation),
      dist_threshold= 0.2)  #FIXME: dist_threshold is too big?

  #print score, parameters
  if score!=None:  return score
  else:            return f_none


def Run(t,*args):
  situation= args[0]
  attr_keys= args[1]
  inferred_keys= args[2] if len(args)>2 else []
  verbose= args[3] if len(args)>3 else True

  a_has= lambda *e: t.HasAttr(*(attr_keys+list(e)))
  a_get= lambda *e: t.GetAttr(*(attr_keys+list(e)))
  a_set= lambda *e_v: t.SetAttr(*(attr_keys+list(e_v)))


  bottle= situation['source']
  cup= situation['receiver']
  arm= situation['handid']
  q_curr= GetOrSet(situation, 'q_curr', lambda:t.robot.Q(arm))
  lw_xe= GetOrSet(situation, 'l_x_ext', lambda:t.GetAttr('wrist_'+LRToStrs(arm),'lx'))

  x_w= t.robot.FK(q=q_curr,arm=arm)

  t.ExecuteMotion('infer2', {},[bottle],'x')
  x_b= a_get(bottle,'x')
  t.ExecuteMotion('infer2', {},[cup],'x')
  x_c= a_get(cup,'x')

  InsertDict(situation, {'source_attr':{'x':x_b}, 'receiver_attr':{'x':x_c}})
  inferred_keys.append([bottle,'l_x_grab'])
  inferred_keys.append([bottle,'l_x_grab0'])
  inferred_keys.append([bottle,'g_width'])


  if not a_has(bottle,'shape_primitives'):  return False

  bb_margin= 1.2  #TEST; bigger than margin for infer_traj
  t.ExecuteMotion('scene', 'make',[],bb_margin)

  #Container to store the variables derived from situation
  dsit= TContainer(debug=True)
  if a_has(bottle,'l_p_pour_e_set'):
    dsit.edge_b_center= TParameterizedPolygon(a_get(bottle,'l_p_pour_e_set')).Center
  elif a_has(bottle,'l_x_pour_e'):
    dsit.edge_b_center= a_get(bottle,'l_x_pour_e')

  dsit.t= t
  dsit.arm= arm
  dsit.bottle= bottle
  dsit.cup= cup
  dsit.x_b    = x_b
  dsit.x_c    = x_c
  dsit.q_curr = q_curr
  dsit.x_w    = x_w
  dsit.lw_xe  = lw_xe
  dsit.idx2angle= [0.0,0.5*math.pi,math.pi,-0.5*math.pi]  #Discretization for cuboid
  dsit.grab_margin= 0.9  #FIXME: magic number

  dsit.shapes= []
  dsit.R= []
  dsit.width= []
  dsit.l_p_1= []
  dsit.l_p_2= []

  primitives= a_get(bottle,'shape_primitives')
  options= {}
  options['param_struct']= ['x_base','x',[]]
  for prm in primitives:
    if prm['kind'] in ('rtpkCylinder','rtpkTube'):
      if prm['kind']=='rtpkCylinder':  height= prm['param'][1]
      elif prm['kind']=='rtpkTube':    height= prm['param'][2]
      dsit.shapes.append( 'cyl' )
      dsit.R.append( QToRot(prm['pose'][3:]) )
      dsit.width.append( 2.0*prm['param'][0] )
      dsit.l_p_1.append( Transform(prm['pose'],[0.,0.,-0.5*height]) )
      dsit.l_p_2.append( Transform(prm['pose'],[0.,0.,+0.5*height]) )

      idx= len(options['param_struct'][2])
      ids= 'c'+str(idx)
      options['param_struct'][2].append([ids,'c'])
      options[ids]= {}
      options[ids]['bounds']= [[-math.pi,-0.5*math.pi,0.05],[math.pi,0.5*math.pi,0.95]]
      options[ids]['tolfun']= 1.0e-2 # 1.0e-4
      options[ids]['scaling_of_variables']= np.array([1.0,0.5,0.2])
      options[ids]['parameters0']= [0.0,0.0,0.9]
      options[ids]['scale0']= 0.7*math.pi
    elif prm['kind'] in ('rtpkCuboid','rtpkRectTube'):
      lenx,leny,lenz= [ l*2.0 for l in prm['param'][0:3] ]
      dsit.shapes.append( 'cub' )
      dsit.R.append( QToRot(prm['pose'][3:]) )
      dsit.width.append( [ leny, lenx, leny, lenx ] )
      dsit.l_p_1.append( Transform(prm['pose'],[0.,0.,-0.5*lenz]) )
      dsit.l_p_2.append( Transform(prm['pose'],[0.,0.,+0.5*lenz]) )

      idx= len(options['param_struct'][2])
      ids= 'x'+str(idx)
      ids2= ['c'+ids+'_0', 'c'+ids+'_1', 'c'+ids+'_2', 'c'+ids+'_3']
      options['param_struct'][2].append([ids,'x',[[ids2[0],'c'],[ids2[1],'c'],[ids2[2],'c'],[ids2[3],'c']]])
      options[ids]= {}
      for i in range(4):
        options[ids2[i]]= {}
        options[ids2[i]]['bounds']= [[-0.5*math.pi,0.05],[0.5*math.pi,0.95]]
        options[ids2[i]]['tolfun']= 1.0e-2 # 1.0e-4
        options[ids2[i]]['scaling_of_variables']= np.array([0.5,0.2])
        options[ids2[i]]['parameters0']= [0.0,0.9]
        options[ids2[i]]['scale0']= 0.7*math.pi
  #print 'options["param_struct"]=',options['param_struct']

  options['maxfevals']= 1000
  options['scale0_ratio1']= 0.005
  options['scale0_ratio2']= 0.2
  options['init_guess_num']= 3
  options['max_init_guess_num']= [200,4000]
  options['db_search_num']= 3
  options['max_db_search_count']= 1000
  options['db_param_novelty']= 0.1
  fobj= lambda x: EvalGrabPoseForFMax(x,dsit,None)

  parameters_res, score_res= CompOptInference(fobj, options,
                      t.database, db_search_key='l_x_grab', infer_type='ver2',
                      param_dist=DistOfGrabParams)
  if score_res is None:
    t.ExecuteMotion('scene', 'clear')
    CPrint(4,'No solution found')
    return False
  InsertDict(situation, {'infer_info':{'type':'ver2', 'param':parameters_res, 'score':score_res}})

  l_x_grab, l_x_grab0, g_width= ExpandGrabParameters(parameters_res, dsit)
  a_set(bottle,'l_x_grab',  l_x_grab)
  a_set(bottle,'l_x_grab0',  l_x_grab0)
  a_set(bottle,'g_width',  g_width)

  t.ExecuteMotion('scene', 'clear')

  return a_has(bottle,'l_x_grab') and a_has(bottle,'g_width')

