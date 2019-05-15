import TrajectoryOptimization: LQRCost, iLQRSolverOptions, AugmentedLagrangianSolverOptions,
    ALTROSolverOptions, Problem, initial_controls!, solve!, goal_constraint, max_violation,
    bound_constraint, Objective, Constraint, Random, Dynamics, Diagonal, I
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

# unconstrained
prob = Problem(model, Objective(costfun,N), x0=x0, N=N, dt=dt)
initial_controls!(prob, U0)
solve!(prob, opts_ilqr)
@test norm(prob.X[N] - xf) < 5.0e-3

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
r_circle = 1.0
circles = ((0.,50.,r_circle),(25.,25.,r_circle),(25.,50.,r_circle))
circles_final = ((0.,50-50,r_circle),(25-25,25.,r_circle),(25,50-25,r_circle))
n_circles = 3

bnd = bound_constraint(n,m,u_min=0.0,u_max=6.0,trim=true)
goal_con = goal_constraint(xf)

global k

function circle_obs_dyn(c,x,u)
    for i = 1:n_circles
        if i == 1  # -y direction
            c[i] = TrajectoryOptimization.circle_constraint(x,circles[i][1],circles[i][2]-(50.0/N)*(k),circles[i][3]+r_quad)
        elseif i == 2  # -x direction
            c[i] = TrajectoryOptimization.circle_constraint(x,circles[i][1]-(25.0/N)*(k),circles[i][2],circles[i][3]+r_quad)
        elseif i == 3  # -x direction
            c[i] = TrajectoryOptimization.circle_constraint(x,circles[i][1],circles[i][2]-(25.0/N)*(k),circles[i][3]+r_quad)
        end
    end
    return nothing
end

# constrained w/ final position, control limits, dynamic obstacles
function get_obs_con(k)  # function call to setup time-vectorized obstacle track
	Constraint{Inequality}(circle_obs_dyn,n,m,n_circles,:obs)
end

con = [[bnd, get_obs_con(k), goal_con] for k = 1:N]  # list of list in list comprehension
# prob_con = ProblemConstraints(con,N)
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
ax.plot3D(x_coord, y_coord, z_coord)

# plotting cylinder obstacles
using VectorizedRoutines
for ii = 1:n_circles
    xc = circles_final[ii][1]
    yc = circles_final[ii][2]
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
end

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

fig.show()
