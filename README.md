# micro_glider_recipe
repository for a micro-underwater glider design recipe

glider_model.m calculates glider flight data (displacements, velocity, pitch and pitch velocity) for every permutation of design parameters (either using variable external or internal parameters). 

It uses glider_ode.m and liftdrag.m to calculate these parameters. 

glider_performance_evaluation.m takes the outputs of glider_model.m and generates flight parameters (glide angle, pitch, angle attack, velocity, depth, etc) and presents them as design spaces, complete flight profiles, whatever you really want).

model_practical_compare.m simply plots model flight profiles against prototype data to compare modelling with reality. 
