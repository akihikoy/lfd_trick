#!/usr/bin/python
from core_tool import *
def Help():
  return '''Filter for cv (color_detector nodes).
  Usage:
    cvfilter1
      Test this filter
    Use CVApplyFilter from other scripts.
  '''

#idx: index of color detection node (1 or 2)
def CVApplyFilter(cvfiltered, msg, idx):
  if 'amount' not in cvfiltered:
    #NOTE: following variables are from two sensors
    cvfiltered.amount= [0.0, 0.0]
    cvfiltered.spill= [0.0, 0.0]
    #Horizontal positions of flow center on camera: [u1,u2]
    cvfiltered.term_flow_center= [0.0, 0.0]
    cvfiltered.term_flow_var= [0.0, 0.0]
    cvfiltered.term_flow_max_dist= [0.0, 0.0]
    cvfiltered.flow_amount= [0.0, 0.0]
    cvfiltered.flow_amount_sum= [0.0, 0.0]
    #Positions of receiver on camera: [u1,v1,u2,v2]
    cvfiltered.uv_rcv= [0.0, 0.0, 0.0, 0.0]
    #Bounds of receiver on camera: [u1,v1,w1,h1,u2,v2,w2,h2]
    cvfiltered.uvbound_rcv= [0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0]
    #Area (number of pixels) of receiver base
    cvfiltered.col_area= [0.0, 0.0]

  alpha= 0.2
  #NOTE: color-0: marker to detect position, color-1: material (colored)
  if msg.col_area[0]>200:  #Enough pixels
    if cvfiltered.col_area[idx-1]==0.0:
      cvfiltered.col_area[idx-1]= msg.col_area[0]
    else:
      cvfiltered.col_area[idx-1]= (1.0-alpha)*cvfiltered.col_area[idx-1] + alpha*msg.col_area[0]
    for i in (0,1):  #u,v
      if cvfiltered.uv_rcv[2*(idx-1)+i]==0.0:
        cvfiltered.uv_rcv[2*(idx-1)+i]= msg.col_center_xy[i]
      else:
        cvfiltered.uv_rcv[2*(idx-1)+i]= (1.0-alpha)*cvfiltered.uv_rcv[2*(idx-1)+i] + alpha*msg.col_center_xy[i]
    for i in (0,1,2,3):  #u,v,w,h
      if cvfiltered.uvbound_rcv[4*(idx-1)+i]==0.0:
        cvfiltered.uvbound_rcv[4*(idx-1)+i]= msg.col_bound[i]
      else:
        cvfiltered.uvbound_rcv[4*(idx-1)+i]= (1.0-alpha)*cvfiltered.uvbound_rcv[4*(idx-1)+i] + alpha*msg.col_bound[i]

  #Sum the material colored blocks
  area_rcv= 0.0
  area_spill= 0.0
  xmin= cvfiltered.uvbound_rcv[4*(idx-1)+0]
  xmax= xmin + cvfiltered.uvbound_rcv[4*(idx-1)+2]
  ymin= cvfiltered.uvbound_rcv[4*(idx-1)+1]  # + cvfiltered.uvbound_rcv[4*(idx-1)+3]
  for i in range(msg.nums_blocks[0], msg.nums_blocks[0]+msg.nums_blocks[1]):
    ap= (msg.blocks_center_xy[2*i+0], msg.blocks_center_xy[2*i+1])
    if ap[0]>=xmin and ap[0]<=xmax and ap[1]<=ymin:
      area_rcv+= msg.blocks_area[i]
    else:
      area_spill+= msg.blocks_area[i]
  div= max(200.0, cvfiltered.col_area[idx-1])
  #am_adjust= 1.0 / 6.0  #1.5 / 6.0  #For b64
  am_adjust= 1.5                     #For b65
  am_rcv= area_rcv / div * am_adjust
  am_spill= area_spill / div / 6.0 * am_adjust
  cvfiltered.amount[idx-1]= (1.0-alpha)*cvfiltered.amount[idx-1] + alpha*am_rcv
  cvfiltered.spill[idx-1]= (1.0-alpha)*cvfiltered.spill[idx-1] + alpha*am_spill

  flow_amount= 0.0
  flow_center= 0.0
  flow_var= 0.0
  for i in range(msg.num_flows):
    spddir= (msg.flows_spddir[2*i+0], abs(msg.flows_spddir[2*i+1]))
    #if spddir[1]>0.5*math.pi-0.2 and spddir[1]<0.5*math.pi+0.2 and spddir[0]>20.0:
    if spddir[0]>2.0:  #TEST:FIXME
      flow_amount+= msg.flows_amount[i]
      flow_center+= msg.flows_xy[2*i+0] * msg.flows_amount[i]
      flow_var= (msg.flows_xy[2*i+0]-cvfiltered.term_flow_center[idx-1])**2 * msg.flows_amount[i]
  if flow_amount>0.0:
    flow_center/= flow_amount
    flow_var/= flow_amount
    cvfiltered.flow_amount[idx-1]= flow_amount/div
    cvfiltered.flow_amount_sum[idx-1]+= cvfiltered.flow_amount[idx-1]
    alpha_f= 0.2 / (1.0 + 0.2*cvfiltered.flow_amount_sum[idx-1])
    #if idx==1: print '###',cvfiltered.flow_amount_sum[idx-1], alpha_f
    if cvfiltered.term_flow_center[idx-1]==0.0:
      cvfiltered.term_flow_var[idx-1]= 0.0
      cvfiltered.term_flow_max_dist[idx-1]= 0.0
      cvfiltered.term_flow_center[idx-1]= flow_center
    else:
      #flow_var= (flow_center-cvfiltered.term_flow_center[idx-1])**2
      cvfiltered.term_flow_var[idx-1]= (1.0-alpha_f)*cvfiltered.term_flow_var[idx-1] + alpha_f*flow_var
      cvfiltered.term_flow_max_dist[idx-1]= max(cvfiltered.term_flow_max_dist[idx-1], math.sqrt(flow_var))
      cvfiltered.term_flow_center[idx-1]= (1.0-alpha_f)*cvfiltered.term_flow_center[idx-1] + alpha_f*flow_center
  else:
    cvfiltered.flow_amount[idx-1]= 0.0

def VizCVFiltered(t, cvfiltered, idx, print_data=False):
  if print_data:
    print cvfiltered.amount[idx-1], cvfiltered.spill[idx-1],
    print '\t', cvfiltered.term_flow_center[idx-1], cvfiltered.flow_amount[idx-1]

  viz_msg= lfd_vision.msg.ColDetViz()
  prim= lfd_vision.msg.ColDetVizPrimitive()
  prim.type= prim.LINE
  prim.color.r= 255.0; prim.color.g= 128.0; prim.color.b= 0.0
  prim.line_width= 2.0
  amt= 200.0
  prim.param= [cvfiltered.term_flow_center[idx-1], cvfiltered.uvbound_rcv[4*(idx-1)+1],
               cvfiltered.term_flow_center[idx-1], cvfiltered.uvbound_rcv[4*(idx-1)+1]-amt ]
  viz_msg.objects.append(prim)

  prim= lfd_vision.msg.ColDetVizPrimitive()
  prim.type= prim.LINE
  prim.color.r= 255.0; prim.color.g= 128.0; prim.color.b= 0.0
  prim.line_width= 2.0
  dev= math.sqrt(cvfiltered.term_flow_var[idx-1])
  prim.param= [cvfiltered.term_flow_center[idx-1]-dev, cvfiltered.uvbound_rcv[4*(idx-1)+1],
               cvfiltered.term_flow_center[idx-1]+dev, cvfiltered.uvbound_rcv[4*(idx-1)+1] ]
  viz_msg.objects.append(prim)

  if   idx==1:  t.pub.coldet_viz.publish(viz_msg)
  elif idx==2:  t.pub.coldet_viz2.publish(viz_msg)

def PassToOldCallbacks(msg, cvfiltered, idx, amount_observer=None, flow_speed_observer=None):
  if amount_observer!=None:
    msg2= std_msgs.msg.Float64()
    msg2.data= cvfiltered.amount[idx-1]
    amount_observer(msg2)
  if flow_speed_observer!=None:
    msg2= std_msgs.msg.Float64MultiArray()
    msg2.data= msg.flow_avr_spddir
    flow_speed_observer(msg2)

def Run(t,*args):
  #t.ExecuteMotion('cv', 'setup')
  #t.ExecuteMotion('cv', 'resume')
  t.ExecuteMotion('cv', 'setup', 0)
  t.ExecuteMotion('cv', 'resume', 0)
  cvfiltered= TContainer(debug=True)
  t.callback.cv= lambda msg: ( CVApplyFilter(cvfiltered, msg, 1), VizCVFiltered(t, cvfiltered, 1, True) )
  t.callback.cv2= lambda msg: ( CVApplyFilter(cvfiltered, msg, 2), VizCVFiltered(t, cvfiltered, 2, True) )
  time.sleep(20.0)
  t.callback.cv= None
  t.callback.cv2= None
  #t.ExecuteMotion('cv', 'pause')
  t.ExecuteMotion('cv', 'pause', 0)


