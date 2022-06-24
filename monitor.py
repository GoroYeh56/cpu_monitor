#!/usr/bin/env python

import functools
import os
import subprocess

import rosnode
import rospy

import psutil


# OverlayText for rviz display
try:
  from jsk_rviz_plugins.msg import *
  print("import * from jsk_rviz_plugins.msg")
except:
  import roslib;roslib.load_manifest("jsk_rviz_plugins")
  from jsk_rviz_plugins.msg import *
  print("go except jsk_rviz_plugins")

from std_msgs.msg import ColorRGBA
import math


try:
  from xmlrpc.client import ServerProxy
except ImportError:
  from xmlrpclib import ServerProxy

from std_msgs.msg import Float32, UInt64


def ns_join(*names):
  return functools.reduce(rospy.names.ns_join, names, "")

class Node:
  def __init__(self, name, pid):
    self.name = name
    self.proc = psutil.Process(pid)
    self.cpu_publisher = rospy.Publisher(ns_join("~", name[1:], "cpu"), Float32, queue_size=20)
    self.mem_publisher = rospy.Publisher(ns_join("~", name[1:], "mem"), UInt64, queue_size=20)
    self.count=1     # later for running average
    self.ave_cpu = 0 # average cpu usage

  def publish(self):
    # self.cpu_publisher.publish(Float32(self.proc.cpu_percent()))
    # Modify heriz_cpu_pub.e: output 'running average' overlay_sample!
    self.count += 1
    self.ave_cpu = self.ave_cpu + (self.proc.cpu_percent()-self.ave_cpu) / self.count
    self.cpu_publisher.publish(self.ave_cpu)
    self.mem_publisher.publish(UInt64(self.proc.memory_info().rss))

  def alive(self):
    return self.proc.is_running()

if __name__ == "__main__":
  rospy.init_node("cpu_monitor")
  master = rospy.get_master()

  poll_period = rospy.get_param('~poll_period', 1.0)

  this_ip = os.environ.get("ROS_IP")

  node_map = {}
  ignored_nodes = set()  #cpu_values.height = 600

  cpu_publish = rospy.Publisher("~total_cpu", Float32, queue_size=20)
  viz_cpu_pub = rospy.Publisher("/viz/cpu_monitor", OverlayText, queue_size=20)
  print("viz_cpu_pub setup!")

  mem_topics = ["available", "used", "free", "active", "inactive", "buffers", "cached", "shared", "slab"]

  vm = psutil.virtual_memory()
  mem_topics = filter(lambda topic: topic in dir(vm), mem_topics)

  mem_publishers = []
  for mem_topic in mem_topics:
    mem_publishers.append(rospy.Publisher("~total_%s_mem" % mem_topic,
                                          UInt64, queue_size=20))

  cpu_values = OverlayText()
  # Set to right upper
  cpu_values.width = 1000
  cpu_values.height = 850
  cpu_values.left = 800
  cpu_values.top = 0
  cpu_values.text_size = 10
  cpu_values.line_width = 1
  cpu_values.font = "DejaVu Sans Mono"


  while not rospy.is_shutdown():
    for node in rosnode.get_node_names():
      if node in node_map or node in ignored_nodes:
        continue

      node_api = rosnode.get_api_uri(master, node)[2]
      if not node_api:
        rospy.logerr("[cpu monitor] failed to get api of node %s (%s)" % (node, node_api))
        continue

      ros_ip = node_api[7:] # strip http://
      ros_ip = ros_ip.split(':')[0] # strip :<port>/
      local_node = "localhost" in node_api or \
                   "127.0.0.1" in node_api or \
                   (this_ip is not None and this_ip == ros_ip) or \
                   subprocess.check_output("hostname").decode('utf-8').strip() in node_api
      if not local_node:
        ignored_nodes.add(node)
        rospy.loginfo("[cpu monitor] ignoring node %s with URI %s" % (node, node_api))
        continue

      try:
        resp = ServerProxy(node_api).getPid('/NODEINFO')
      except:
        rospy.logerr("[cpu monitor] failed to get pid of node %s (api is %s)" % (node, node_api))
      else:
        try:
          pid = resp[2]
        except:
          rospy.logerr("[cpu monitor] failed to get pid for node %s from NODEINFO response: %s" % (node, resp))
        else:
          node_map[node] = Node(name=node, pid=pid)
          rospy.loginfo("[cpu monitor] adding new node %s" % node)

    # Clear every time
    cpu_values.text = ""
    for node_name, node in list(node_map.items()):
      if node.alive():
        node.publish()
        # cpu.text add this node
        this_node_cpu_text = node.name + ": " + str(node.ave_cpu) +"\n"
        cpu_values.text += this_node_cpu_text
        
      else:
        rospy.logwarn("[cpu monitor] lost node %s" % node_name)
        del node_map[node_name]

    cpu_publish.publish(Float32(psutil.cpu_percent()))

    # Add total cpu values
    cpu_values.text = "Total cpu: " + str(Float32(psutil.cpu_percent())) + " %\n" + cpu_values.text
    # Set colors  
    cpu_values.fg_color = ColorRGBA(25 / 255.0, 1.0, 240.0 / 255.0, 1.0)
    cpu_values.bg_color = ColorRGBA(0.0, 0.0, 0.0, 0.2)
    # Publich cpu_values
    viz_cpu_pub.publish(cpu_values)
    
  


    vm = psutil.virtual_memory()
    for mem_topic, mem_publisher in zip(mem_topics, mem_publishers):
      mem_publisher.publish(UInt64(getattr(vm, mem_topic)))

    rospy.sleep(poll_period)
