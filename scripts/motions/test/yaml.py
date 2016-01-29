#!/usr/bin/python
from core_tool import *
import yaml
def Help():
  return '''Test of loading yaml file.
  Usage: test_yaml'''
def Run(t,*args):
  data= yaml.load(file('models/obj/b50.yaml').read())
  print data
