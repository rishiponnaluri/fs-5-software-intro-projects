import matplotlib.pyplot as plt
from pid_template import make_car
from pid_template import update
from pid_template import calculate_desired_acceleration
from pid_template import acceleration_to_throttle_percentage

K_P = 8.0
K_I = 0.02
K_D = 0.5
 
STEPS = 550
 
car = make_car(desired_v=20.0, dt=0.1)

#WRITE CODE HERE
v = []
e = []
t = []
for i in range (STEPS):
    a, error = calculate_desired_acceleration(car, K_P, K_I, K_D)
    e.append(error)
    v.append(car["v"])
    t.append(car["t"])
    throttle = acceleration_to_throttle_percentage(a)
    if 20 <= car["t"] < 30:
        friction = 4.0
    else:
        friction = 2.0
    update(car, throttle, friction=friction)
# Velocity vs Time
plt.figure()
plt.plot(t, v)
plt.xlabel("Time (s)")
plt.ylabel("Velocity (m/s)")
plt.title("Velocity vs Time")

# Error vs Time
plt.figure()
plt.plot(t, e)
plt.xlabel("Time (s)")
plt.ylabel("Error (m/s)")
plt.title("Error vs Time")

plt.show()


    
    
