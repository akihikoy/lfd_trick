#!/usr/bin/python
#\file    test_tostdtype.py
#\brief   Test ToStdType.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Dec.09, 2015
import roslib; roslib.load_manifest('lfd_trick')
from base import *

class TXYAR(object):
  def __init__(self,x=None,y=None,a=None,r=None):
    self.x= x  #State
    self.y= y  #Output
    self.a= a  #Action
    self.r= r  #Reward

class TChainEpisode(object):
  def __init__(self, len=None):
    self.Seq= []  #Each element should have an interface of TXYAR.
    self.R= None  #Sum of rewards.
    if len is not None:  self.Seq= [TXYAR() for i in range(len)]

  @property
  def Len(self):
    return len(self.Seq)

'''Database of episodes where each episode is an instance of TChainEpisode.'''
class TChainEpisodeDB(object):
  def __init__(self):
    self.Entry= []  #List of episodes.

  #Save into data (dict).
  def Save(self):
    ST= ToStdType
    data= {}
    data['entry']= ST(self.Entry)
    return data

  #Load from data (dict).
  def Load(self, data):
    if data is None:  return
    if 'entry' in data:
      self.Entry= [TChainEpisode() for i in range(len(data['entry']))]
      for i,eps in enumerate(data['entry']):
        self.Entry[i].Seq= [None]*len(eps['Seq'])
        for n,xyar in enumerate(eps['Seq']):
          self.Entry[i].Seq[n]= TXYAR(x=xyar['x'],y=xyar['y'],a=xyar['a'],r=xyar['r'])
        self.Entry[i].R= eps['R']


if __name__=='__main__':
  randi= lambda N: int(random.random()*N)
  gen_ref= lambda Dx,Dy,Da: [TXYAR(
    x=RandVec(Dx) if Dx>0 else None,
    y=RandVec(Dy) if Dy>0 else None,
    a=RandVec(Da) if Da>0 else None,
    r=Rand(0.0,1.0) if Dy>0 else None) for i in range(3)]
  refs= [gen_ref(1,0,2)] + [gen_ref(1+randi(3),randi(3),randi(2)) for n in range(3)] + [gen_ref(0,3,0)]
  #print ToStdType(refs)
  #PrintDict( {'refs': ToStdType(refs)} )
  print yamldump( ToStdType(refs) )

  print '=================================='

  N= 4
  Dx= [1,1+randi(3),1+randi(3),1+randi(3),0]
  Dy= [0,randi(3),randi(3),randi(3),3]
  Da= [2,randi(2),randi(2),randi(2),0]
  db= TChainEpisodeDB()
  for i in range(20):
    eps= TChainEpisode(N+1)
    for n,s in enumerate(eps.Seq):
      s.x=RandVec(Dx[n]) if Dx[n]>0 else None
      s.y=RandVec(Dy[n]) if Dy[n]>0 else None
      s.a=RandVec(Da[n]) if Da[n]>0 else None
      s.r=Rand(0.0,1.0) if Dy[n]>0 else None
    eps.R= sum([s.r if s.r is not None else 0.0 for s in eps.Seq])
    db.Entry.append(eps)

  print yamldump( db.Save() )
  db2= TChainEpisodeDB()
  db2.Load(db.Save())
  print '----------------------------------'
  print yamldump( db2.Save() )
  print type(db2.Entry)
  print type(db2.Entry[0])
  print type(db2.Entry[0].R)
  print type(db2.Entry[0].Seq)
  print type(db2.Entry[0].Seq[0])

  print '=================================='

  state_bounds= [TBoundingBox(Dx[n]) for n in range(N+1)]
  for i in range(10):
    for n in range(N+1):
      state_bounds[n].Add(RandVec(Dx[n]))

  print yamldump( ToStdType({'state_bounds':state_bounds}) )

