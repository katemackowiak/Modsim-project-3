from modsim import *
import math

m = UNITS.meter
s = UNITS.second
kg = UNITS.kilogram

def make_system(omega):
    """Makes a System object for the given conditions.
    
    params: Params object
    
    returns: System object
    """ 

    mass = 78 * kg #mass of ryan gosling
    grip = 111.2 * kg #ryan goslings grip strenth
    r = 20 * m #radius of ferris wheel in the movie

    pos = Vector(r,0*m) 
    init = State(pos)#initializes state values, these should probably change at some point


    t_0 = 0 * s
    t_end = 10 * s
    dt = t_end/100

    g = 9.81 *m*(s**-2)

    return System(init=init, mass = mass, grip = grip, r=r, t_0 = t_0, t_end= t_end, dt = dt, g = g, omega = omega)

def slope_func(state, t, system):
    """Compute derivatives of the state.
    SHOULD FIX THE DOCSTRINGS AT SOME POINT
    """
    pos = state
    pos = pos[0]
    mass, grip, r, omega, t_0, t_end, dt, g = system.mass, system.grip, system.r, system.omega, system.t_0, system.t_end, system.dt, system.g
    
    unit_pos = (pos.hat()) 

    V = Vector((-1*(unit_pos.y) *  omega * r), (unit_pos.x * omega * r)) 

   
    f_c = Vector(mass * V[0]**2  / r, mass *V[1]**2  / r)
    print(f_c)
    f_g = Vector(0*kg*m*s**-2, -1*mass * g)
    print(f_g) 
    
    f_t = f_c + f_g
    force = sqrt((f_t[0]**2) + (f_t[1]**2))

    force_array.append(force)

    dposdt = [V] 

    return dposdt 

def event_func(state, t, system):
    """Return the height of the penny above the sidewalk.
    """
    V = state
    return V

force_array = []

system = make_system(.05/s)

results, details = run_ode_solver(system, slope_func)#, events=event_func)
details
print(results)
print(force_array)
'''x = results.extract('x')
y = results.extract('y')'''
#plot(results[0], results[1])