#!/usr/bin/python
from core_tool import *
def Help():
  return '''Infer grab pose l_x_grab, its width g_width, and pre grab pose l_x_grab0.
  Usage: infer_grab SITUATION, ATTR_KEYS [, INFERRED_KEYS [, VERBOSE]]
    SITUATION: dictionary of situation.  e.g. {'p':[0,0,0]}
    ATTR_KEYS: list of attributes keys. e.g. ['b1']
    INFERRED_KEYS: list of inferred keys is stored
    VERBOSE: if True, inferred information is printed (default:True)'''
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


  #Evaluate a grab pose of a cylinder model for pouring
  #l_x_g: grab point and orientation in the source container frame
  #l_x_g0: pre-grab point and orientation in the source container frame
  #g_width: grab width
  #x_b: source container pose
  #x_c: receiving container pose
  #l_p_1, l_p_2: cylinder (bottom and top points) of source container
  #width: width of the cylinder
  #edge_b_center: center of pouring edge points
  def EvalGrabPose(l_x_g, l_x_g0, g_width, x_b, x_c, l_p_1, l_p_2, edge_b_center, arm, q_curr, x_w, lw_xe, verbose=False):
    score= 1.0
    p_g,R_g= XToPosRot(Transform(x_b,l_x_g))
    p_1= Transform(x_b,l_p_1)
    p_2= Transform(x_b,l_p_2)
    e_x= R_g[:,0]
    e_y= R_g[:,1]
    e_z= R_g[:,2]

    p_ec= Transform(x_b,edge_b_center)

    #TEST
    ##Requirement from pouring:
    #a1= GetAngle(e_x, (Vec(x_c)-Vec(x_b))[0:3])
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
    p_w, R_w= XToPosRot(x_w)
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
    ##if p_g[2]-x_b[2] <= 0.05:  # Height of grab point is less than 5cm
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
    x_g0= Transform(x_b,l_x_g0)
    x_w_trg= TransformRightInv(x_g0,lw_xe)  #Target of wrist x
    vg,res2,res3= t.ExecuteMotion('scene','isvalidx',arm,x_w_trg,[],q_curr)
    if verbose:  print 'res2.error_code.val=',res2.error_code.val if res2 else None
    if verbose:  print 'res3.error_code.val=',res3.error_code.val if res3 else None
    if verbose and res3:
      #print 'res3:\n',res3
      VisualizeContacts(res3.contacts,dt=rospy.Duration(0.5))
    if not vg:  return None

    #Check the validity of grab pose
    x_g= Transform(x_b,l_x_g)
    x_w_trg= TransformRightInv(x_g,lw_xe)  #Target of wrist x
    ignore= [bottle]  #Ignoring collision with bottle
    vg,res2,res3= t.ExecuteMotion('scene','isvalidx',arm,x_w_trg,ignore,q_curr)
    if verbose:  print 'res2.error_code.val=',res2.error_code.val if res2 else None
    if verbose:  print 'res3.error_code.val=',res3.error_code.val if res3 else None
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
  def ExpandGrabParameters(parameters, l_p_1, l_p_2, width):
    a_1= parameters[0]
    a_2= parameters[1]
    ez= Normalize(Vec(l_p_2)-Vec(l_p_1))
    ex= GetOrthogonalAxisOf(ez, preferable=[1.0,0.0,0.0], fault=[0.0,0.0,-1.0])
    ex= np.dot(RFromAxisAngle(ez,a_1),ex)
    ey= np.cross(ez,ex)
    R= Eye()
    R[:,0]= ex
    R[:,1]= ey
    R[:,2]= ez
    l_q_g= RotToQ(np.dot(RFromAxisAngle(ey,a_2),R))

    grab_ratio= parameters[2]
    l_p_g= grab_ratio*Vec(l_p_1) + (1.0-grab_ratio)*Vec(l_p_2)
    l_x_g= list(l_p_g)+list(l_q_g)

    #Estimating pre-grab pose l_x_g0
    #Method A
    #l.x_grab0= copy.deepcopy(l.x_grab)
    #l.x_grab0[0]-= t.GetAttr(obj,'g_width')  #TODO: generalize
    #l.x_grab0[3:7]= t.robot.FK(arm=handid)[3:7]
    #Method B
    #p_g,R_g= XToPosRot(l.x_grab)
    #p_g0= Vec(p_g) - np.dot(R_g[:,0], 1.5*t.GetAttr(obj,'g_width'))  #TODO: generalize
    #l.x_grab0= PosRotToX(p_g0,R_g)

    g_width= 0.9*width  #FIXME: magic number
    l_p_g0= Vec(l_p_g) - np.dot(ex, 1.5*width)  #FIXME: magic number
    l_x_g0= list(l_p_g0)+list(l_q_g)

    return l_x_g, l_x_g0, g_width

  #Distance to modify the score function from examples
  def DistOfGrabPoseExs(parameters, x_b, x_c, l_p_1, l_p_2, width, arm, q_curr, lw_xe,  situation):
    if situation['infer_info']['type']!='ver1':  return None
    dists= []
    dists.append(Norm(Vec(parameters) - Vec(situation['infer_info']['param'])))
    d_x_b= DiffX(x_b, situation['source_attr']['x'])
    dists.append(Norm(d_x_b[:3]))
    dists.append(0.2*Norm(d_x_b[3:]))
    d_x_c= DiffX(x_c, situation['receiver_attr']['x'])
    dists.append(Norm(d_x_c[:3]))
    dists.append(0.2*Norm(d_x_c[3:]))
    #print dists
    dist= max(dists)
    return dist

  def EvalGrabPoseForFMin(parameters, x_b, x_c, l_p_1, l_p_2, width, edge_b_center, arm, q_curr, x_w, lw_xe, f_none=100):
    l_x_g, l_x_g0, g_width= ExpandGrabParameters(parameters, l_p_1, l_p_2, width)
    score= EvalGrabPose(l_x_g, l_x_g0, g_width, x_b, x_c, l_p_1, l_p_2, edge_b_center, arm, q_curr, x_w, lw_xe)
    score= ModifyScoreFromExamples(
        score= score,
        bad_exs= t.GetAttrOr([], 'memory','grab','bad'),
        good_exs= t.GetAttrOr([], 'memory','grab','good'),
        dist_func= lambda situation, inferred_data, assessment: DistOfGrabPoseExs(parameters, x_b, x_c, l_p_1, l_p_2, width, arm, q_curr, lw_xe,  situation),
        dist_threshold= 0.2)  #FIXME: dist_threshold is too big?

    #print score, parameters
    if score!=None:  return -score
    else:            return f_none


  if a_has(bottle,'grab_primitives'):
    bb_margin= 1.2  #TEST; bigger than margin for infer_traj
    t.ExecuteMotion('scene', 'make',[],bb_margin)

    if a_has(bottle,'l_p_pour_e_set'):
      edge_b_center= TParameterizedPolygon(a_get(bottle,'l_p_pour_e_set')).Center
    elif a_has(bottle,'l_x_pour_e'):
      edge_b_center= a_get(bottle,'l_x_pour_e')

    primitives= a_get(bottle,'grab_primitives')
    for prm in primitives:
      #FIXME:how to select from multiple primitives?
      if prm['kind']=='pkCylinder':
        if prm['width']<0.08:  #TODO:provide by attributes
          #x_bl= [0.0,0.0,0.0, 0.0,0.0,0.0,1.0]
          #x_cl= TransformLeftInv(x_b, x_c)
          l_p_1= prm['p1']
          l_p_2= prm['p2']
          width= prm['width']

          fobj= lambda x: EvalGrabPoseForFMin(x,x_b,x_c,l_p_1,l_p_2,width,edge_b_center,arm,q_curr,x_w,lw_xe,None)
          options= {'CMA_diagonal':1, 'verb_time':0}
          options['bounds']= [[-math.pi,-0.5*math.pi,0.05],[math.pi,0.5*math.pi,0.95]]
          options['tolfun']= 1.0e-2 # 1.0e-4
          options['maxfevals']= 1000
          options['verb_log']= False
          options['scaling_of_variables']= np.array([1.0,0.5,0.2])
          parameters0= [0.0,0.0,0.9]
          scale0= 0.7*math.pi

          options['scale0_ratio1']= 0.005
          options['scale0_ratio2']= 0.2
          options['init_guess_num']= 3
          options['max_init_guess_num']= [200,4000]
          options['db_search_num']= 3
          options['max_db_search_count']= 1000
          options['db_param_novelty']= 0.1

          #options['scaling_of_variables']= np.array([1.0,0.5,1.0,0.2])
          ##typical_x= [2.30496913,1.3967557,-0.46143515,0.77916813] #previous solution
          ##options['typical_x']= np.array(typical_x)
          #scale0= 0.05
          #parameters0= [2.30496913,1.3967557,-0.46143515,0.77916813] #previous solution
          ##print 'typical_x, score=',typical_x,EvalGrabPoseForFMin(typical_x,x_bl,x_cl,l_p_1,l_p_2,width,edge_b_center,arm,q_curr,x_w,lw_xe,None)

          parameters_res,score_res= ExecInference(fobj, parameters0, scale0, options,
                              t.database, db_search_key='l_x_grab', infer_type='ver1')
          if score_res is None:
            t.ExecuteMotion('scene', 'clear')
            CPrint(4,'No solution found')
            return False
          InsertDict(situation, {'infer_info':{'type':'ver1', 'param':parameters_res, 'score':score_res}})

          l_x_grab, l_x_grab0, g_width= ExpandGrabParameters(parameters_res, l_p_1, l_p_2, width)
          a_set(bottle,'l_x_grab',  l_x_grab)
          a_set(bottle,'l_x_grab0',  l_x_grab0)
          a_set(bottle,'g_width',  g_width)

    t.ExecuteMotion('scene', 'clear')

  elif a_has(bottle,'l_x_grab_set'):  #DEPRECATED:Estimate from l_x_grab_set:
    candidates= a_get(bottle,'l_x_grab_set')
    l_x_grab_avr= np.average(candidates,0)
    #Find a grab point whose z position is closest to the average
    i_closest= -1
    d_closest= 1.0e20
    for i in range(len(candidates)):
      if abs(candidates[i][2]-l_x_grab_avr[2]) < d_closest:
        d_closest= abs(candidates[i][2]-l_x_grab_avr[2])
        i_closest= i
    if i_closest>=0:
      a_set(bottle,'l_x_grab',  candidates[i_closest])

  return a_has(bottle,'l_x_grab') and a_has(bottle,'g_width')

