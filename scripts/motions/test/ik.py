#!/usr/bin/python
from core_tool import *
def Help():
  return '''Test IK.
  Usage: test_ik'''

def testXTrajToQTraj(func_ik, x_traj, start_angles):
  N= len(x_traj)
  q_prev= start_angles
  q_traj= None
  for x,n in zip(x_traj, range(N)):
    q= func_ik(x, q_prev)
    if q==None:  return None
    if q_traj==None:  q_traj= [[0.0]*len(q) for i in range(N)]
    q_traj[n][:]= q
    q_prev= q
  SmoothQTraj(q_traj)
  return q_traj

def Run(t,*args):
  x_traj= [
    [0.58960843838981103, -0.16494943434637574, -0.20602815496471155, -0.00088241563912943124, 0.010741884074064208, 0.088536880960485287, 0.99601458522401576],
    [0.58743068755528938, -0.15626813771509823, -0.2075851767394819, -0.0017668089677402689, 0.021476036613030366, 0.08850832304400838, 0.99584232436134423],
    [0.58417893525406661, -0.14330532561509729, -0.20991017381246069, -0.0026509972852526687, 0.032207697187139818, 0.088469495093328882, 0.99555451126155148],
    [0.58016231710494481, -0.12729319264356859, -0.2127822050050345, -0.0035348779951655321, 0.042935620551681583, 0.088420401613835284, 0.99515117932093589],
    [0.57568996872672562, -0.10946393339770791, -0.21598032913858992, -0.0044183485366709081, 0.053658561895590602, 0.088361048302073364, 0.99463237533998672],
    [0.571071025738211, -0.091049742474710968, -0.21928360503451366, -0.0053013063965545986, 0.064375276985888907, 0.088291442045085142, 0.99399815951795367],
    [0.56661462375820304, -0.073282814471773566, -0.22247109151419234, -0.0061836491210912553, 0.075084522312059851, 0.088211590919609853, 0.99324860544586147],
    [0.56262989840550326, -0.057395343986091397, -0.2253218473990127, -0.0070652743279325586, 0.0857850552303384, 0.088121504191146546, 0.99238380009797045],
    [0.55913809083876254, -0.043472027745658698, -0.22782067245935428, -0.0079460797179871038, 0.096475634107900901, 0.088021192312879221, 0.99140384382168545],
    [0.55566493108263459, -0.029622554119509165, -0.23030647004395013, -0.0088259630872906895, 0.10715501846693763, 0.08791066692446374, 0.99030885032591132],
    [0.55219187007725745, -0.015773048774969382, -0.23279245961389497, -0.009704822338865441, 0.11782196912859101, 0.087789940850677234, 0.98909894666785847],
    [0.5487188971797069, -0.0019235285208292728, -0.23527861221183227, -0.010582555494566695, 0.12847524835674348, 0.08765902809993005, 0.9877742732383008],
    [0.54524600174705806, 0.011925989834121395, -0.23776489888040545, -0.011459060706915936, 0.13911362000163788, 0.087517943862640224, 0.98633498374528472],
    [0.54177317313638618, 0.025775489481092699, -0.24025129066225781, -0.012334236270918644, 0.14973584964331349, 0.087366704509470913, 0.98478124519629395],
    [0.5383004007047667, 0.039624953611294852, -0.24273775860003288, -0.013207980635865618, 0.16034070473484166, 0.087205327589430809, 0.98311323787887073],
    [0.53482767380927498, 0.053474365415938002, -0.24522427373637404, -0.014080192417116366, 0.17092695474534403, 0.087033831827837818, 0.98133115533969661],
    [0.53135498180698615, 0.067323708086232212, -0.24771080711392468, -0.014950770407863233, 0.18149337130277651, 0.086852237124146345, 0.97943520436213394],
    [0.52788231405497565, 0.08117296481338776, -0.25019732977532821, -0.015819613590874925, 0.19203872833646349, 0.086660564549638205, 0.97742560494223163],
    [0.52440965991031863, 0.095022118788614732, -0.25268381276322804, -0.016686621150217983, 0.20256180221936404, 0.086458836344977641, 0.97530259026319865],
    [0.52093938077436097, 0.10886626769589114, -0.25517135249683914, -0.017551692482954968, 0.21306137191005545, 0.086247075917630631, 0.97306640666834621]]
  t_traj= [0.15, 0.3, 0.45, 0.6, 0.75, 0.9, 1.05, 1.2, 1.35, 1.5, 1.65, 1.8, 1.95, 2.1, 2.25, 2.4, 2.55, 2.7, 2.85, 3.0]

  q0= t.robot.IK(x_traj[0])
  q_traj= testXTrajToQTraj(lambda x,q0:t.robot.IK(x,start_angles=q0), x_traj, q0)
  #print q_traj