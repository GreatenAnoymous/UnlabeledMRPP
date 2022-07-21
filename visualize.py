#!/usr/bin/env python3
import matplotlib
# matplotlib.use("Agg")
from matplotlib.patches import Circle, Rectangle, Arrow
from matplotlib.collections import PatchCollection
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation
import argparse
import math
import json
from ast import literal_eval as make_tuple
scale=5
PURPLE=(102/255.,0,255/255.)
BLUE=(0.0,193.0/255.0,232.0/255.)
GREEN=(0,176.0/255.0,80.0/255.0)
ORANGE=(255./255.,192.0/255.0,0)
PINK=(255.0/255.0,102.0/255.0,153.0/255.0)
RED=(1.0,0,0)


def load_map(input_map):
    with open(input_map,"r") as file_content:
        lines = file_content.readlines()
        xmax=int(lines[1].split()[1])
        ymax=int(lines[2].split()[1])
        print(xmax,ymax)
        obstacles = list()
        for y, line in enumerate(lines[4:]):
            for x, char in enumerate(line):
                if char != "." and x<xmax and y<ymax:
                    obstacles.append((x, y))
     
        return xmax,ymax,obstacles



def load_paths_from_json(file_name):
    f1=open(file_name)
    paths_info=json.load(f1)
    paths=paths_info["paths"]
    return paths

def color_for_goal(g,m):
    start_color=[1,0,0]
    start_color[1]=start_color[1]+g[0]/m
    start_color[2]=start_color[2]+g[1]/m
    start_color[0]=start_color[0]-(g[0]+g[1])/(2.5*m)
    
    return np.array(start_color)
    

class Animation:
  def __init__(self, xmax,ymax,obstacles, paths):
    
    #self.schedule = schedule
    self.paths=paths

    #aspect = map["map"]["dimensions"][0] / map["map"]["dimensions"][1]
    aspect=1
    sizex=16
    self.fig = plt.figure(frameon=False, figsize=(sizex * aspect, sizex))
    self.ax = self.fig.add_subplot(111, aspect='equal')
    self.fig.subplots_adjust(left=0,right=1,bottom=0,top=1, wspace=None, hspace=None)
    # self.ax.set_frame_on(False)

    self.patches = []
    self.artists = []
    self.agents = dict()
    self.agent_names = dict()
    # create boundary patch
    xmin = -0.5
    ymin = -0.5
    self.xmax = xmax - 0.5
    self.ymax =ymax - 0.5

    # self.ax.relim()
    plt.xlim(xmin, xmax)
    plt.ylim(ymin, ymax)
    # self.ax.set_xticks([])
    # self.ax.set_yticks([])
    # plt.axis('off')
    # self.ax.axis('tight')
    # self.ax.axis('off')
    print("starting!")
   
    self.patches.append(Rectangle((xmin, ymin), xmax - xmin, ymax - ymin, facecolor='none', edgecolor='red'))
    for o in obstacles:
          x, y = o[0], o[1]
          self.patches.append(Rectangle((x - 0.5, y - 0.5), 1, 1, facecolor='black', edgecolor='red'))


    # create agents:
    self.T = 0
    # draw goals first
    for  i in range(0,len(paths)):
      goali=self.paths[i][-1]
      self.patches.append(Rectangle((goali[0] - 0.25, goali[1] - 0.25), 0.5, 0.5, facecolor=GREEN, edgecolor='black', alpha=0.15))
      #self.patches.append(Rectangle((d["goal"][0] - 0.25, d["goal"][1] - 0.25), 0.5, 0.5, facecolor=self.colors[i%len(self.colors)], edgecolor='black', alpha=0.15))
    for i in range(0,len(paths)):
      #name = d["name"]
      starti=self.paths[i][0]
      self.agents[i] = Circle((starti[0], starti[1]), 0.3, facecolor=ORANGE, edgecolor='black')
      
      self.patches.append(self.agents[i])
      self.T = max(self.T, len(paths[i])-1)
      #self.agent_names[name] = self.ax.text(d["start"][0], d["start"][1], name.replace('agent', ''),fontsize=10)
      #self.agent_names[name].set_horizontalalignment('center')
      #self.agent_names[name].set_verticalalignment('center')
      #self.artists.append(self.agent_names[name])

    print("agents created!")

    # self.ax.set_axis_off()
    # self.fig.axes[0].set_visible(False)
    # self.fig.axes.get_yaxis().set_visible(False)

    # self.fig.tight_layout()
    self.init_func()
    self.anim = animation.FuncAnimation(self.fig, self.animate_func,
                               init_func=self.init_func,
                               frames=int(self.T+1) * scale,
                               interval=scale,
                               blit=True,repeat=False)
    self.anim.save("exmaple.gif",fps=10, writer='pillow')
    


        
  def arrange_random_colors(self,m=30):
      
      self.colors=[]

      
      for i in range(len(self.paths)):
          si=self.paths[i][0]
          gi=self.paths[i][-1]
         
          self.colors.append(color_for_goal(gi,m))     
        
  
  def save(self, file_name, speed):
    self.anim.save(
      file_name,
      "ffmpeg",
      fps=10 * speed,
      dpi=800),
      # savefig_kwargs={"pad_inches": 0, "bbox_inches": "tight"})

  def show(self):
    plt.show()

  def init_func(self):
    for p in self.patches:
      self.ax.add_patch(p)
    for a in self.artists:
      self.ax.add_artist(a)
    return self.patches + self.artists

  def animate_func(self, i):
    for k in range(0,len(self.paths)):
      #agent = schedule["schedule"][agent_name]
      path=self.paths[k]
      #pos = self.getState(i / scale, agent)
      pos = self.getState(i / scale, path)
      p = (pos[0], pos[1])
      self.agents[k].center = p
      #self.agent_names[agent_name].set_position(p)

    return self.patches + self.artists


  def getState(self, t, d):
    idx = 0
    while idx < len(d) and idx < t:
      idx += 1
    if idx == 0:
      return np.array([float(d[0][0]), float(d[0][1])])
    elif idx < len(d):
      posLast = np.array([float(d[idx-1][0]), float(d[idx-1][1])])
      posNext = np.array([float(d[idx][0]), float(d[idx][1])])
    else:
      return np.array([float(d[-1][0]), float(d[-1][1])])
    #dt = d[idx]["t"] - d[idx-1]["t"]
    #t = (t - d[idx-1]["t"]) / dt
    t=t-(idx-1)
    pos = (posNext - posLast) * t + posLast
    return pos



if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("map", help="input file containing map")
  parser.add_argument("paths", help="paths file for agents")
  parser.add_argument('--video', dest='video', default=None, help="output video file (or leave empty to show on screen)")
  parser.add_argument("--speed", type=int, default=1, help="speedup-factor")
  args = parser.parse_args()


  #with open(args.map) as map_file:
  #  map = yaml.load(map_file)
 
  xmax,ymax,obstacles=load_map(args.map)
  print(obstacles)
  paths=load_paths_from_json(args.paths)
  


  print(args.paths)
  print("number of agents=",len(paths))
  animation = Animation(xmax,ymax,obstacles, paths)

  if args.video:
    animation.save(args.video, args.speed)
  else:
    animation.show()
