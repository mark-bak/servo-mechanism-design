import control as ct

def dc_brushed_motor(params, additional_outputs=[]):
    # Motor tfs
    tf_circ = ct.tf(
        1,
        [params["l_a"], params["r_a"]],
        inputs = 'u_a',
        outputs = 'i',
        name = 'circ',
    )
    
    tf_i = ct.tf(
        params["kT"],
        1,
        inputs = 'i',
        outputs = 'torque_motor',
        name = 'torque_gain',
    )
    

    tf_load = ct.tf(
        1,
        [params["j_rotor"],params["b"]],
        inputs = 'torque_acc',
        outputs = ['omega_motor'],
        name = 'load',
    )
    
    tf_acc_torque = ct.summing_junction(
        ['torque_motor','-torque_out_motor'],
        'torque_acc'
    )

    volts = ct.summing_junction(
    ['u_s','-u_emf'],
    'u_a',
    name = 'sum',
    )
    
    tf_emf = ct.tf(
        params["kT"],
        1,
        inputs = 'omega_motor',
        outputs = 'u_emf',
        name = 'emf_gain'
    )

    motor = ct.interconnect(
        [tf_circ, tf_i, tf_load, tf_emf, volts, tf_acc_torque],
        inputs = ['u_s', 'torque_out_motor'],
        outputs = ['omega_motor'] + additional_outputs,
        params = {
            "speed_out": 'omega_motor',
            "torque_out": "torque_out_motor",
            # "load": "torque_load_motor",
        },
        name = 'dc_motor_brushed',
    )   
    
    return motor

def sat_ud(t,x,u,params):
    return x

def sat_out(t,x,u,params):
    if u > params["max"]:
        return params["max"]
    if u < params["min"]:
        return params["min"]
    return u

def saturating_voltage_supply(params, additional_outputs=[]):
    u_sat = ct.nlsys(
        sat_ud,
        sat_out,
        inputs = 'u',
        outputs = ['u_s'] + additional_outputs,
        states=1,
        name = 'sat_v',
        params = params,
    )
    
    return u_sat

def transmission(params, input_connect, output_names, additional_outputs = []):
    tf_speed = ct.tf(
        1 / params["n"],
        1,
        inputs = input_connect.params["speed_out"],
        outputs = output_names['speed'],
    )
    
    
    tf_inertial_torque = ct.tf(
        [params["j"], 0 ],
        [1e-6, 1],
        inputs = input_connect.params["speed_out"],
        outputs = 'torque_acc',
    )
    
    tf_torque_bk = ct.tf(
        1,
        params["n"] * params["eff"],
        inputs = output_names["torque"],
        outputs = 'torque_fb',
    )
 
    tf_fb_torque = ct.summing_junction(
        ['torque_acc', 'torque_fb'], 
        input_connect.params['torque_out']
    )
    
    trans = ct.interconnect(
        [tf_speed, tf_fb_torque, tf_torque_bk, tf_inertial_torque],
        inputs = [input_connect.params["speed_out"], output_names["torque"]],
        outputs = [output_names['speed'], input_connect.params["torque_out"]] + additional_outputs,
        params = {
            "speed_out": output_names["speed"],
            "torque_out": output_names["torque"],
        },
    )
    return trans
    

def linear_load(params, input_connect, additional_outputs = []):
    tf_posn = ct.tf(
        1,
        [1,0],
        inputs = input_connect.params["speed_out"],
        outputs ='position'
    )
    
    tf_inertial_torque = ct.tf(
        [params["m"] ,params["c"]],
        [1e-6, 1],
        inputs = input_connect.params['speed_out'],
        outputs = 'force_acc'
    )
    
    tf_stiffness_torque = ct.tf(
        params["k"],
        1,
        inputs = 'position',
        outputs = 'stiffness_force'
    )
    
    tf_fb_torque = ct.summing_junction(
        ['force_acc', 'stiffness_force'],
        input_connect.params["torque_out"]
    )
    
    load = ct.interconnect(
        [tf_posn, tf_inertial_torque, tf_stiffness_torque, tf_fb_torque],
        inputs = input_connect.params["speed_out"],
        outputs = ['position', input_connect.params["torque_out"]] + additional_outputs,
    )
    
    return load

def pi_controller(params, set_name, fb_name, control_name):
    tf_error = ct.summing_junction(
        [set_name, '-'+fb_name],
        'err'
    ) 
    
    tf_control = ct.tf(
        [params["kp"], params["ki"]],
        [1, 0],
        inputs = 'err',
        outputs = control_name,
    )
    
    controller = ct.interconnect(
        [tf_error, tf_control],
        inputs = [set_name, fb_name],
        outputs = control_name,
    )
    
    return controller
       
    # tf_out_torque = ct.tf(
    #     1,
    #     1,
    #     inputs = 'torque_load_motor',
    #     outputs = 'torque_out_motor',
    #     )
    



# tf_theta = ct.tf(
#     1,
#     [1,0],
#     inputs = 'omega',
#     outputs = 'theta',
#     name = 'posn_integrator',
# )


# tf_pressure = ct.tf(
#     k_hyd,
#     1,
#     inputs = 'theta',
#     outputs = 'pressure',
#     name = 'pressure_gain',
# )


# torque_sum = ct.summing_junction(
#     ['motor_torque', '-fb_torque'],
#     'sum_torque',
#     name = 'torque_sum',
# )


# tf_torque_eff = ct.tf(
#     eff,
#     1,
#     inputs = 'sum_torque',
#     outputs = 'eff_torque',
#     name = 'efficiency',
# )