from robot import Robot 
from roboticstoolbox import VehicleIcon
import matplotlib.pyplot as plt

def main():
    anim = VehicleIcon('../car.png', scale=2)
    veh = Robot(animation=anim)
    plt.plot()
    veh.go((3, 4))
    plt.pause(50)
    


if __name__ == "__main__":
    main()
