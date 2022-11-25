from robot import Robot 
import matplotlib.pyplot as plt

def main():
    veh = Robot(animPath="../car.png",)
    plt.plot()
    veh.go((3, 4))
    plt.pause(50)
    


if __name__ == "__main__":
    main()
