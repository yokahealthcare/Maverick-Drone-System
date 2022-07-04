#!/usr/bin/env python
import os
from rich.console import Console
import pyfiglet
import cowsay

console = Console()

import square
import fly_circle
import hexagonal
import takeoff_hover_land
import landing
import takeoff

from pyfiglet import Figlet
custom_fig = Figlet(font='chunky')
console.print(custom_fig.renderText('MAVERICK'), style="bold red")
console.print("DRONE SYSTEM - Created by CPS Drone Team")

menus = [
    'Square',
    'Circle',
    'Hexagonal',
    'Take Off & Land',
    'Takeoff',
    'Land',
    'Exit'        
]

running = True
while running:
    for i in range(len(menus)):
        console.print("[{}] {}".format(i+1, menus[i]), style="bold")
    
    u = int(input(">>>>  "))
    if u == 1:
        square.run()
        
    elif u == 2:
        fly_circle.run()
        
    elif u == 3:
        hexagonal.run()
    
    elif u == 4:
        takeoff_hover_land.run()
        
    elif u == 5:
        takeoff.run()
    
    elif u == 6:
        landing.run()
        
    elif u == len(menus):
        cowsay.cow("BYE BYE moooo....")
        running = False
        
    else:
        print("Invalid Input - System Cannot Understand!!")
    
        
        
