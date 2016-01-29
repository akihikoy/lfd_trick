#!/usr/bin/python
from core_tool import *
def Help():
  return '''Infer attributes (ver.2).
  Usage: infer2 SITUATION, ATTR_KEYS, INFER_TYPE [, INFERRED_KEYS [, VERBOSE]]
    SITUATION: dictionary of situation.  e.g. {'p':[0,0,0]}
    ATTR_KEYS: list of attributes keys. e.g. ['b1']
    INFER_TYPE: inference type. e.g. 'x'
    INFERRED_KEYS: list of inferred keys is stored
    VERBOSE: if True, inferred information is printed (default:True)'''
def Run(t,*args):
  situation= args[0]
  attr_keys= args[1]
  infer_type= args[2]
  inferred_keys= args[3] if len(args)>3 else []
  verbose= args[4] if len(args)>4 else True

  a_has= lambda *e: t.HasAttr(*(attr_keys+list(e)))
  a_get= lambda *e: t.GetAttr(*(attr_keys+list(e)))
  a_getor= lambda df,*e: t.GetAttrOr(df,*(attr_keys+list(e)))
  a_set= lambda *e_v: t.SetAttr(*(attr_keys+list(e_v)))

  situation['infer_type']= infer_type
  start_time= time.time()

  def ExitProc(inferred,store=True):
    end_time= time.time()

    #Add result to the database:
    if inferred and store:
      situation['taskid']= t.GetAttrOr('', CURR,'taskid')
      situation['time']= TimeStr('normal')
      situation['rostime']= rospy.Time.now().to_nsec()
      situation['environment']= t.GetAttrOr('', 'environment')

      inferred_data= []
      assessment= {}
      assessment['infer_time']= end_time-start_time
      for k in inferred_keys:
        keys= attr_keys+k
        value= ToStdType(a_get(*k))
        inferred_data.append([keys, value])
      t.database.append([ToStdType(situation), inferred_data, assessment])
      #TEST: append to items recently added to the database
      new_idx= len(t.database)-1
      t.SetAttr(CURR,'new_in_database', t.GetAttrOr([],CURR,'new_in_database')+[new_idx])

    if verbose:
      print 'In situation:',situation
      if inferred:
        for k in inferred_keys:
          print '  Inferred: [%s]= %r' % (']['.join(attr_keys+k), a_get(*k))
      else:
        for k in inferred_keys:
          print '  Not inferred: [%s]' % (']['.join(attr_keys+k))
      print '  Took %f [sec]' % (end_time-start_time)

  #Infer x (pose)
  if infer_type=='x':
    inferred_keys.append(['x'])

    #Check the inference candidates and priority:
    method= ''
    #Infer x when obj is grabbed:
    if a_has('grabbed'):
      method= 'grab'
    #Infer x when obj is on a kinematics chain:
    elif a_has('kinematics_chain'):
      method= 'fk'
    else:
      score_ref= None
      score_base= None
      #Infer x when reference marker is known:
      if a_has('ref_marker_id') and a_has('ref_marker_pose'):
        if a_get('ref_marker_id') in t.ar_markers and len(a_get('ref_marker_pose'))==7:
          score_ref= t.ARXtime(a_get('ref_marker_id'))
      #Infer x when base marker is known:
      if a_has('base_marker_id'):
        if a_get('base_marker_id') in t.ar_markers:
          score_base= t.ARXtime(a_get('base_marker_id'))
      if score_ref!=None and score_base!=None:  #When both are observed, use new one
        if score_ref>=score_base:  method= 'refm'
        else:  method= 'basem'
      elif score_ref!=None:  method= 'refm'
      elif score_base!=None:  method= 'basem'
      #print 'score_ref:',score_ref
      #print 'score_base:',score_base

    #Infer x when obj is grabbed:
    if method=='grab':
      grabber_handid= a_getor(None,'grabbed','grabber_handid')
      grabber_wrist= a_getor(None,'grabbed','grabber_wrist')
      lo_x_grab= a_getor(None,'l_x_grab')

      if None not in (grabber_handid,grabber_wrist,lo_x_grab):
        x_w= t.robot.FK(arm=grabber_handid)

        lw_xe= t.GetAttr(grabber_wrist,'lx')
        a_set('x',  TransformRightInv(Transform(x_w, lw_xe), lo_x_grab))
      else:
        method= ''
        if verbose:
          print 'Some of following are missing:'
          print '  grabber_handid=',grabber_handid
          print '  grabber_wrist=',grabber_wrist
          print '  l_x_grab=',lo_x_grab

    #Infer x when obj is on a kinematics chain:
    elif method=='fk':
      parent_arm_id=  StrToLR(a_get('kinematics_chain','parent_arm'))
      if parent_arm_id != None:
        a_set('x',  t.robot.FK(arm=parent_arm_id))
      else:
        method= ''
        if verbose:
          print 'Some of following are missing:'
          print '  [kinematics_chain][parent_arm]=',parent_arm_id

    #Infer x when reference marker is known:
    elif method=='refm':
      ref_marker_id= a_getor(None,'ref_marker_id')
      ref_marker_pose= a_getor(None,'ref_marker_pose')
      if None not in (ref_marker_id,ref_marker_pose):
        if verbose:  print '###infer from ref',ref_marker_id
        x_m_ref= t.ARX(ref_marker_id)
        a_set('x',  TransformRightInv(x_m_ref, ref_marker_pose))
      else:
        method= ''
        if verbose:
          print 'Some of following are missing:'
          print '  ref_marker_id=',ref_marker_id
          print '  ref_marker_pose=',ref_marker_pose

    #Infer x when base marker is known:
    elif method=='basem':
      base_marker_id= a_getor(None,'base_marker_id')
      if None is not base_marker_id:
        if verbose:  print '###infer from base',base_marker_id
        a_set('x',  t.ARX(base_marker_id))
      else:
        method= ''
        if verbose:
          print 'Some of following are missing:'
          print '  base_marker_id=',base_marker_id

    #If 'x' is inferred in some way, show the result and exit:
    if method!='':
      ExitProc(inferred=True,store=False)
      return True

  #Infer x (pose)
  elif infer_type=='bound_box':
    inferred_keys.append(['bound_box'])
    if a_has('bound_box'):
      pass  #Do nothing if already exists
    else:
      ##Get a bounding box
      res= True
      if not a_has('grab_primitives'):
        ExitProc(inferred=False)
        return False
      bpmax= [0.0,0.0,0.0]
      bpmin= [0.0,0.0,0.0]
      for prm in a_get('grab_primitives'):
        if prm['kind']=='pkCylinder':
          w= prm['width']
          p1= Vec(prm['p1'])
          p2= Vec(prm['p2'])
          h= Norm(p2-p1)
          axis= np.cross([0.0,0.0,1.0],p2-p1)
          if Norm(axis)>1.0e-6:
            angle= GetAngle([0.0,0.0,1.0],p2-p1)
            R= RFromAxisAngle(Normalize(axis),angle)
          else:
            R= Eye()
          c= 0.5*(p1+p2)
          n_div= 40
          for i in range(n_div):
            angle= 2.0*math.pi*float(i)/float(n_div)
            p= [0.5*w*math.cos(angle),0.5*w*math.sin(angle),0.5*h]
            p= np.dot(R,p) + c
            bpmax= [max(d) for d in zip(bpmax,p)]
            bpmin= [min(d) for d in zip(bpmin,p)]
            p= [0.5*w*math.cos(angle),0.5*w*math.sin(angle),-0.5*h]
            p= np.dot(R,p) + c
            bpmax= [max(d) for d in zip(bpmax,p)]
            bpmin= [min(d) for d in zip(bpmin,p)]
        elif prm['kind']=='pkCube':
          x_center= Vec(prm['x_center'])
          dims= Vec(prm['dims'])
          corners= [[1.,1.,1.],[-1.,1.,1.],[-1.,-1.,1.],[1.,-1.,1.],
                    [1.,1.,-1.],[-1.,1.,-1.],[-1.,-1.,-1.],[1.,-1.,-1.]]
          for lp in corners:
            p= Transform(x_center, [0.5*dims[d]*lp[d] for d in range(3)])
            bpmax= [max(d) for d in zip(bpmax,p)]
            bpmin= [min(d) for d in zip(bpmin,p)]
      if a_has('l_p_pour_e_set'):
        l_p_pour_e_set= a_get('l_p_pour_e_set')
        for p in l_p_pour_e_set:
          bpmax= [max(d) for d in zip(bpmax,p)]
          bpmin= [min(d) for d in zip(bpmin,p)]
      center= list(0.5*(Vec(bpmin)+Vec(bpmax)))+[0.0,0.0,0.0,1.0]
      dim= [d[1]-d[0] for d in zip(bpmin,bpmax)]
      a_set('bound_box', {'center':center,'dim':dim})
      ExitProc(inferred=True,store=False)
      return True

  elif infer_type=='grab':
    res= t.ExecuteMotion('infer_grab2', situation,attr_keys,inferred_keys,verbose)
    ExitProc(inferred=res)
    return res

  elif infer_type=='obspose':
    res= t.ExecuteMotion('infer_obspose', situation,attr_keys,inferred_keys,verbose)
    ExitProc(inferred=res)
    return res

  elif infer_type=='pour':
    res= t.ExecuteMotion('infer_pour', situation,attr_keys,inferred_keys,verbose)
    ExitProc(inferred=res)
    return res

  elif infer_type=='poursp':
    res= t.ExecuteMotion('infer_poursp', situation,attr_keys,inferred_keys,verbose)
    ExitProc(inferred=res)
    return res

  elif infer_type=='regrab':
    res= t.ExecuteMotion('infer_regrab', situation,attr_keys,inferred_keys,verbose)
    ExitProc(inferred=res)
    return res

  elif infer_type=='spread':
    res= t.ExecuteMotion('infer_spread', situation,attr_keys,inferred_keys,verbose)
    ExitProc(inferred=res)
    return res

  elif infer_type=='traj':
    res= t.ExecuteMotion('infer_traj', situation,attr_keys,inferred_keys,verbose)
    ExitProc(inferred=res)
    return res

  ExitProc(inferred=False)
  return False

