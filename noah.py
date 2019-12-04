from modsim import *
import math

m = UNITS.meter
s = UNITS.second
kg = UNITS.kilogram

def make_system(omega):
    """Makes a System object for the given conditions.
    """ 

    mass = 78 * kg #mass of ryan gosling
    grip = 1054 * kg * m * s**-2 #ryan goslings grip strenth approxiamately
    r = 20 * m #radius of ferris wheel in the movie

    pos = Vector(r,0*m) 
    init = State(pos)#initializes state values, these should probably change at some point


    t_0 = 0 * s
    t_end = 30 * s
    dt = t_end/50

    g = 9.81 *m*(s**-2)

    return System(init=init, mass = mass, grip = grip, r=r, t_0 = t_0, t_end= t_end, dt = dt, g = g, omega = omega)

def slope_func(state, t, system):
    """Calculates the change in position based on the angular velocity at the given time
	Also calculates and stores the force on Noah at each position based on where he is and the speed

    """
    pos = state
    pos = pos[0]
    mass, grip, r, omega, t_0, t_end, dt, g = system.mass, system.grip, system.r, system.omega, system.t_0, system.t_end, system.dt, system.g
    
    unit_pos = (pos.hat()) 

    V = Vector((-1*(unit_pos.y) *  omega * r), (unit_pos.x * omega * r)) 
 
    f_c = Vector(mass * V[0]**2  / r, mass *V[1]**2  / r)
    f_g = Vector(0*kg*m*s**-2, -1*mass * g)
    
    f_t = f_c + f_g
    force = sqrt((f_t[0]**2) + (f_t[1]**2))

    if int(force *s**2 / kg / m) > int(grip * s**2 / kg /m):
        global launched
        if launched == False:
            launched = True
            print('he has launched at the speed of {} or {}'.format(omega, V.mag))
        
    force_array.append(force)

    dposdt = [V] 

    return dposdt 

def event_func(state, t, system):
    pos = state
    pos[1] = pos[1] - system.r
    return pos

force_array = []
speed = 0.7000000000000004/s
speed_step = .025/s
speed_end = 0.7100000000000004/s
system = make_system(speed)
launched = False
while speed < speed_end:
    results, details = run_ode_solver(system, slope_func)
    speed += speed_step
    system = make_system(speed)

time = linspace(0,len(force_array)*system.dt/2,len(force_array))

plot(time, force_array, label = 'Force to Hang On')
plot([0,time[-1]],[system.grip,system.grip], label = 'Grip Strength')
plot
decorate(xlabel='Time (Seconds)',
         ylabel='Force (Newtons)',
         title='Force Required Over Time to Not Fall Off the Ferris Wheel',
         legend=True)
plt.show()