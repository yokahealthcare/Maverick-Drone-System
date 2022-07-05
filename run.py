#!/usr/bin/env python
from rich.console import Console
import cowsay

console = Console()

import square
import circle
import triangle
import hexagonal
import takeoff_hover_land

from pyfiglet import Figlet
custom_fig = Figlet(font='chunky')

menus = [
    'Square',
    'Circle',
    'Triangle',
    'Hexagonal',
    'Take Off & Land',
    'Exit'        
]

stop = False
while not stop or not KeyboardInterrupt:
    # Print Banner
    console.print(custom_fig.renderText('MAVERICK'), style="bold red")
    console.print("ドローンシステム-CPSドローンチームによって作成されました")
    
    # Print menus
    for i in range(len(menus)):
        console.print("[{}] {}".format(i+1, menus[i]), style="bold")
    
    u = int(input(">>>>  "))
    if u == 1:
        square.run()
        
    elif u == 2:
        circle.run()
    
    elif u == 3:
        triangle.run()
        
    elif u == 4:
        hexagonal.run()
    
    elif u == 5:
        takeoff_hover_land.run()
        
    elif u == len(menus):
        cowsay.cow("BYE BYE moooo....")
        stop = True
        
    else:
        print("Invalid Input - System Cannot Understand!!")
    
        
        
