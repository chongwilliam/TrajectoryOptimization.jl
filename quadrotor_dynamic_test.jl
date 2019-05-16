import TrajectoryOptimization: LQRCost, iLQRSolverOptions, AugmentedLagrangianSolverOptions,
    ALTROSolverOptions, Problem, initial_controls!, solve!, goal_constraint, max_violation,
    bound_constraint, BoundConstraint, Objective, Constraint, Random, Dynamics, Diagonal, I
Random.seed!(7)

# model
T = Float64
integration = :rk4
model = Dynamics.quadrotor_model
n = model.n; m = model.m

# cost
Q = (1.0e-1)*Diagonal(I,n)
R = (1.0e-1)*Diagonal(I,m)
Qf = 1000.0*Diagonal(I,n)

# -initial state
x0 = zeros(n)
x0[1:3] = [0.; 0.; 0.]
q0 = [1.;0.;0.;0.]
x0[4:7] = q0

# -final state
xf = copy(x0)
xf[1:3] = [0.;50.;0.] # xyz position
xf[4:7] = q0

costfun = LQRCost(Q, R, Qf, xf)

# options
verbose=false
opts_ilqr = iLQRSolverOptions{T}(verbose=verbose,cost_tolerance=1.0e-6)
opts_al = AugmentedLagrangianSolverOptions{T}(verbose=verbose,opts_uncon=opts_ilqr,constraint_tolerance=1.0e-5,cost_tolerance=1.0e-6,cost_tolerance_intermediate=1e-5)
opts_altro = ALTROSolverOptions{T}(verbose=verbose,opts_al=opts_al)

N = 101
dt = 0.1
U0 = [0.5*9.81/4.0*ones(m) for k = 1:N-1]

# # unconstrained
# prob = Problem(model, Objective(costfun,N), x0=x0, N=N, dt=dt)
# initial_controls!(prob, U0)
# solve!(prob, opts_ilqr)
# @test norm(prob.X[N] - xf) < 5.0e-3

# # constrained w/ final position
# goal_con = goal_constraint(xf)
# con = [goal_con]
# prob = Problem(model, Objective(costfun,N),constraints=ProblemConstraints(con,N), x0=x0, N=N, dt=dt)
# initial_controls!(prob, U0)
# solve!(prob, opts_al)
# @test norm(prob.X[N] - xf) < opts_al.constraint_tolerance
# @test max_violation(prob) < opts_al.constraint_tolerance
#
# # constrained w/ final position and control limits
# bnd = bound_constraint(n,m,u_min=0.0,u_max=6.0,trim=true)
# con = [bnd,goal_con]
# prob = Problem(model, Objective(costfun,N), constraints=ProblemConstraints(con,N), x0=x0, N=N, dt=dt)
# initial_controls!(prob, U0)
# solve!(prob, opts_al)
# @test norm(prob.X[N] - xf) < opts_al.constraint_tolerance
# @test max_violation(prob) < opts_al.constraint_tolerance
#
# # constrained w/ final position, control limits, static obstacles
# r_quad = 1.0
# r_sphere = 3.0
# spheres = ((0.,10.,0.,r_sphere),(0.,20.,0.,r_sphere),(0.,30.,0.,r_sphere))
# n_spheres = 3
#
# function sphere_obs3(c,x,u)
#     for i = 1:n_spheres
#         # c[i] = TrajectoryOptimization.sphere_constraint(x,spheres[i][1],spheres[i][2],spheres[i][3],spheres[i][4]+r_quad)
#         c[i] = TrajectoryOptimization.circle_constraint(x,spheres[i][1],spheres[i][2],spheres[i][4]+r_quad)
#     end
#     return nothing
# end
#
# obs = Constraint{Inequality}(sphere_obs3,n,m,n_spheres,:obs)
# con = [bnd,obs,goal_con]
# prob_con = ProblemConstraints(con,N)
# prob = Problem(model, Objective(costfun,N), constraints=ProblemConstraints(con,N),x0=x0, N=N, dt=dt)
# initial_controls!(prob, U0)
# opts_al.constraint_tolerance=1.0e-3
# opts_al.constraint_tolerance_intermediate=1.0e-3
# solve!(prob, opts_al)
# @test norm(prob.X[N] - xf) < opts_al.constraint_tolerance
# @test max_violation(prob) < opts_al.constraint_tolerance

# constrained with circles
r_quad = 1.0
r_circle = 3.0
circles = ((0.,25.,r_circle),(25.,25.,r_circle),(25.,50.,r_circle))
circles_final = ((0.,50-25,r_circle),(25,25.,r_circle),(25,50-25,r_circle))
n_circles = 3

# bnd = bound_constraint(n,m,u_min=0.0,u_max=6.0,trim=true)
bnd = BoundConstraint(n,m,u_min=0.0,u_max=6.0,trim=true)
goal_con = goal_constraint(xf)

# compute dynamic obstacle path
x_obs = zeros(N,1)
y_obs = zeros(N,1)
x0_new = circles[1][1]
y0_new = circles[1][2]

# Generate vector of constraints
function gen_obs(k)
	x_obs[k] = x0_new
	y_obs[k] = y0_new - (25.0/N)*(k-1)

	function obs_avoid(c,x,u)
		c[1] = TrajectoryOptimization.circle_constraint(x,x_obs[k],y_obs[k],circles[1][3]+r_quad)
	end
	Constraint{Inequality}(obs_avoid,n,m,n_circles,:obs)
end

con = [[bnd, gen_obs(k), goal_con] for k = 1:N]
ProblemConstraints(con)
model

# con = [[bnd, get_obs_con(kk), goal_con] for kk = 1:N]  # list of list in list comprehension
# prob_con = ProblemConstraints(con,N)
x0
n
Problem(model, Objective(costfun, N), constraints=ProblemConstraints(con), x0=x0, dt=dt, N=N)
prob = Problem(model, Objective(costfun,N), constraints=ProblemConstraints(con), x0=x0, N=N, dt=dt)
initial_controls!(prob, U0)
opts_al.constraint_tolerance=1.0e-3
opts_al.constraint_tolerance_intermediate=1.0e-3
solve!(prob, opts_al)
@test norm(prob.X[N] - xf) < opts_al.constraint_tolerance
@test max_violation(prob) < opts_al.constraint_tolerance

# using Plots
# plot(to_array(prob.X)[1:3,:]')
# plot(to_array(results_obs.U[1:solver_obs.N-1])')

using PyPlot
quad_position = to_array(prob.X)[1:3,:]'
# plot(to_array(results_obs.U[1:solver_obs.N-1])')
fig = figure("Quad Trajectory")
ax = fig[:add_subplot](111, projection="3d")
x_coord = quad_position[:,1]
y_coord = quad_position[:,2]
z_coord = quad_position[:,3]
# ax.plot3D(x_coord, y_coord, z_coord)
ax.scatter(x_coord, y_coord, z_coord)
# fig.savefig("/home/william/Desktop/hello.png")

# plotting time history of the first moving cylinder
x0_circle = circles[1][1]
y0_circle = circles[1][2]
x_hist = zeros(N,1)
y_hist = zeros(N,1)
for i = 1:N
	x_hist[i] = x0_circle
	y_hist[i] = y0_circle - (25.0/N)*(i-1)
end

# plotting cylinder obstacles
using VectorizedRoutines
for j = 1:N
	fig = figure("Quad Trajectory")
	ax = fig[:add_subplot](111, projection="3d")
	ax.scatter(x_coord[j], y_coord[j], z_coord[j])
	# for ii = 1:n_circles
	    # xc = circles_final[ii][1]
	    # yc = circles_final[ii][2]
		xc = x_hist[j]
		yc = y_hist[j]
	    xset = collect(-r_circle+xc:0.5:r_circle+xc)
	    zset = collect(0.:10.:50.)
	    Xc, Zc = Matlab.meshgrid(xset, zset)
	    ones_mat = ones(size(Xc,1),size(Xc,2))
	    Yc1 = yc*ones_mat + sqrt.( r_circle^2*ones_mat - (Xc-xc*ones_mat).^2 )
	    Yc2 = yc*ones_mat - sqrt.( r_circle^2*ones_mat - (Xc-xc*ones_mat).^2 )

	    # Draw parameters
	    rstride = 20
	    cstride = 10
	    ax.plot_surface(Xc, Yc1, Zc, alpha=0.2, rstride=rstride, cstride=cstride, color="r")
	    ax.plot_surface(Xc, Yc2, Zc, alpha=0.2, rstride=rstride, cstride=cstride, color="r")
	# end

	ax.set_xlim([x_coord[j]-10, x_coord[j]+10])
	ax.set_ylim([y_coord[j]-10, y_coord[j]+10])
	# ax.set_xlim([-10, 10])
	# ax.set_ylim([0, 50])
	ax.set_zlim([0, 50])
	ax.set_xlabel("X")
	ax.set_ylabel("Y")
	ax.set_zlabel("Z")
	# ax.view_init(azim=0, elev=90)
	filename = string("/home/william/Desktop/trajopt_data/",string(j),".png")
	fig.savefig(filename)
	# sleep(3)
	# fig.close()
end

fig.show()
