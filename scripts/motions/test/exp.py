#!/usr/bin/python
from core_tool import *
def Help():
  return '''Test of experiments script.
  Usage: test_exp'''
def Run(t,*args):

  timestamp= TimeStr('short2')
  log_filen= t.LogFileName('test_exp',timestamp)
  log_filep= file(log_filen,'w')
  CPrint(2,'Do you want to save memory/database? (timestamp:%s)'%timestamp)
  if t.AskYesNo():
    t.ExecuteMotion('attr', 'savemem', t.LogFileName('memory',timestamp,'-before.yaml') )
    t.ExecuteMotion('db', 'save', t.LogFileName('database',timestamp,'-before.yaml') )

  def ExitProc():
    log_filep.close()
    CPrint(2,'test_exp: logged to ',log_filen)
    CPrint(2,'Do you want to save memory/database? (timestamp:%s)'%timestamp)
    if t.AskYesNo():
      t.ExecuteMotion('attr', 'savemem', t.LogFileName('memory',timestamp,'-after.yaml') )
      t.ExecuteMotion('db', 'save', t.LogFileName('database',timestamp,'-after.yaml') )

  is_real= (t.GetAttr('environment')=='real')
  containers= ('b53','b54','b55','b56','b57','b58','b59','b60','b61','b62','b63')
  exit_loop= False
  for i in range(10):
    i_src= 0
    #for src in containers:
    while i_src < len(containers):
      try:
        src= containers[i_src]
        rcv= 'b51'
        command= ('pour', src,rcv,0.3, 80)
        #rcv= 'b100'
        #command= ('spread', src,rcv,0.8, 80)
        t.ExecuteMotion('viz')

        if not is_real:
          if not t.ExecuteMotion('randx', 'markercf',src,rcv):  continue
        else:
          CPrint(1,'Setup the containers:',src,rcv)
          CPrint(2,src,'is:',t.GetAttr(src,'help'))
          CPrint(1,rcv,'is:',t.GetAttr(rcv,'help'))
          CPrint(1,'Ready?')
          CPrint(1,'#Answer y: Robot moves. Press "-" to force quit.')
          CPrint(1,'#Answer n: Skip this setup.')
          CPrint(1,'#Answer b: Go back to previous setup.')
          CPrint(1,'#Answer x: Quit now.')
          res= t.AskGen('y','n','b','x')
          if res=='y':  pass
          elif res=='n':
            i_src+=1
            continue
          elif res=='b':
            i_src= i_src-1 if i_src>0 else 0
            continue
          elif res=='x':
            exit_loop= True
            break

        timestampl= TimeStr('normal')
        time_start= rospy.Time.now().to_nsec()
        exec_status= t.ExecuteMotion(*command)
        log_filep.write('%s %f %f %i %s %s\n'%(
                        timestampl,
                        time_start,
                        rospy.Time.now().to_nsec(),
                        i,
                        ','.join(map(str,command)),
                        exec_status ) )
        log_filep.flush()
        #if not IsSuccess(exec_status):
        if exec_status=='failure.forcedquit':
          CPrint(4,'Result:',exec_status)
          CPrint(2,'Do you want to continue?')
          CPrint(1,'#Answer n: Move to the next setup.')
          CPrint(1,'#Answer a: Do again the same setup.')
          CPrint(1,'#Answer x: Quit now.')
          res= t.AskGen('n','a','x')
          if res=='n':
            i_src+=1
          elif res=='a':
            pass
          elif res=='x':
            exit_loop= True
            break
        else:
          i_src+=1
        if not IsSuccess(exec_status):
          t.ExecuteMotion('release',src)
          t.ExecuteMotion('init0',True)
      except Exception as e:
        PrintException(e,' exception in pouring @%i'%i)
        log_filep.write('%s %f %f %i %s exception:%s\n'%(
                        timestampl,
                        time_start,
                        rospy.Time.now().to_nsec(),
                        i,
                        ','.join(map(str,command)),
                        type(e) ) )
        CPrint(2,'Do you want to continue?')
        if t.AskYesNo():
          t.ExecuteMotion('release',src)
          t.ExecuteMotion('init0',True)
        else:
          ExitProc()
          raise e

    if exit_loop:
      break

  ExitProc()

'''
Findings:

Difficult setups:

NOTE: This will fail without memory.
pour 'b53','b51',0.3, 25
[b53][x]= [0.84150253414363307, -0.13092875168802873, -0.28134154988199678, -2.3694374580562251e-05, 0.00079336194585558658, 0.040434512745851317, 0.99918187543329917]
[b51][x]= [0.6269782168147235, 0.0018139474491461805, -0.28, 0.0, 0.0, 0.56758995782056099, 0.82331138688910033]

spread 'b53','b100',0.8, 60
[b53][x]= [0.74207487496910196, 0.13749023672287675, -0.28008494732707367, 4.2179940061864842e-05, 7.7724538629507929e-05, -0.7415156965642612, 0.6709355139866775]
[b100][x]= [0.5953662568998562, -0.05529703466564478, -0.28, 0.0, 0.0, 0.49089365724212675, 0.87121950005693138]

pour 'b54','b51',0.3, 25
[b54][x]= [0.80982132066189916, 0.13246985020481794, -0.28117616040688465, -0.00023318000623335152, 0.00011644815291333, 0.99962158307059445, -0.02750677600794714]
[b51][x]= [0.5966976300739659, -0.056790745662800825, -0.28, -0, -0, -0.032811375351530378, 0.99946156186595847]

pour 'b54','b51',0.3, 25
Left arm joint angles:  (0.16675603811990936, 0.39998500964578376, 1.14878913076471, -0.6269229058974632, -11.866492345931338, -0.9581372977257381, 17.27849087412439)
[b51][x]= [0.6298953280155823, 0.020191275753419814, -0.28, -0, -0, -0.88333560306693437, 0.46874109309337852]
[b54][x]= [0.7999120530370506, -0.068474066933813638, -0.28102231390862542, -0.00055667396025309225, 0.00052606983857851467, 0.45844305410243619, 0.88872345502403494]

pour 'b54','b51',0.3, 25
[b51][x]= [0.6204719532532608, -0.09924158680286332, -0.28, -0, -0, -0.038088445850606091, 0.99927437187825718]
[b54][x]= [0.7343527585665709, 0.086087007464848306, -0.28007969625701651, 0.00054127029803745805, 0.00022869555025291304, 0.99955935256872219, -0.029677523771476449]

pour 'b54','b51',0.3, 25
[b54][x]= [0.74738140799283592, -0.18961819575257133, -0.27988934503015683, -3.6115225276614382e-05, -6.2307837512983618e-05, -0.079847812635829615, 0.99680706339326142]
[b51][x]= [0.6191779565083833, -0.03330726933366295, -0.28, 0.0, 0.0, 0.80899115360002638, 0.58782081742389702]

pour 'b54','b51',0.3, 25
[b51][x]= [0.6020695685324472, -0.12312358519809413, -0.28, 0.0, 0.0, 0.16331260329430822, 0.9865743730734323]
[b54][x]= [0.8349460484960614, 0.06695484915707524, -0.28, -0.0, -0.0, -0.999998292008565, 0.0018482369849921227]
'''

