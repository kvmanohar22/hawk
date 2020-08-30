import numpy as np

def writeToFile(data, path):
    with open(path, 'w') as f:
        f.write("[ ")
        for idx, i in enumerate(data):
            if idx == len(data)-1:
                f.write("{}]".format(i))
            else:
                f.write("{}, ".format(i))

def save(xyz, path):
    writeToFile(xyz[0], path+"_x.txt")
    writeToFile(xyz[1], path+"_y.txt")
    writeToFile(xyz[2], path+"_z.txt")

def circle(radius, altitude, n_waypoints=30):
    theta = np.linspace(0.0, 2*np.pi, n_waypoints)
    xs = radius * np.cos(theta)
    ys = radius * np.sin(theta)
    x = xs.tolist()
    y = ys.tolist()
    z = np.zeros_like(xs) + altitude
    return [x,y,z]

def lamniscate(x_max, altitude, n_waypoints=30):
    theta = np.linspace(0.0, 4.0*np.pi/2.0, n_waypoints)
    cos_t = np.cos(theta)
    sin_t = np.sin(theta)
    xs = x_max * cos_t / (1 + np.power(sin_t, 2)) 
    ys = x_max * sin_t * cos_t / (1+np.power(sin_t, 2))
    x = xs.tolist()
    y = ys.tolist()
    z = np.zeros_like(xs) + altitude
    return [x,y,z]

if __name__ == '__main__':
    data = circle(2, 1.0)
    save(data, "circle/circle")
    # data = lamniscate(2, 1.0)
    # save(data, "lamniscate/lam")

