
# Installing libraries
using Pkg

Pkg.add("JuMP")
Pkg.add("Gurobi")
Pkg.add("CSV")
Pkg.add("XLSX")
Pkg.add("DataFrames")
Pkg.add("Plots")
Pkg.add("GLPK")

# Load libraries
using JuMP, Gurobi, CSV, XLSX, DataFrames, Plots

# Read the data from the CSV file
data = CSV.read("my_data.csv", DataFrame)

# Access the columns from the read DataFrame
criti = data.criti      # criticality
labels = data.labels    # labels or load type
pawar = data.pawar      # power rating
start_time = data.start_time   # appliance start_time
end_time = data.end_time       # appliance end_time
d = data.d                     # duration of operation

println("\nData read from $data:")

# Data and variables for appliances

# Data for the solar plant 6kW (8.55kWh/day), Brookings OCT 12, 2019,  from renewable energy ninja: https://www.renewables.ninja/
P_PV = [0, 0, 0, 0, 0, 0, 0.101, 0.544, 0.973, 1.393, 1.43, 1.361, 1.365, 1.152, 0.682, 0.292, 0.066, 0, 0, 0, 0, 0, 0, 0] # data read from csv file

C = criti
appliance_names = labels
power = pawar

L = length(appliance_names)  # Number of loads i.e. 10
T = 24  # Total time, 24 hours

# Constants and Assumptions
λ = 0.2
E_total = 3.5               # Battery capacity in kWh
SOC_min = 0.1 * E_total     # Minimum SOC
SOC_max = 0.9 * E_total     # Maximum SOC
ηc = 0.95                   # Charging efficiency
ηd = 0.95                   # Discharging efficiency
initial_SOC = 0.8 * E_total # Initial SOC (80% of E_total)
final_SOC = 0.2 * E_total   # Final SOC (20% of E_total)

# Power limits
Pbc_max = 1.0   # Max charging power in kW
Pbc_min = 0.0   # Min charging power in kW
Pbd_max = -1.0  # Max discharging power in kW
Pbd_min = 0.0   # Min discharging power in kW
times = 1:T

# Create model
model = Model(Gurobi.Optimizer)

# Decision variables
@variable(model, x[1:L, 1:T], Bin)  # Binary decision variable for appliances
@variable(model, y[1:L, 1:T], Bin)  # Auxiliary binary variable for non-interruptible devices

@variable(model, P_bc[1:T])             # Charging power, battery
@variable(model, P_bd[1:T])             # Discharging power, battery

@variable(model, P_curtail[1:T])        # Curtailed power
@variable(model, SOC[1:T])              # State of charge

@variable(model, Bc[1:T], Bin)          # Binary charging status
@variable(model, Bd[1:T], Bin)          # Binary discharging status

@variable(model, skip[i=1:L], Bin)      # Binary variable for skipping appliance,


# Objective function: Maximize served load and minimize curtailment
@objective(model, Max, 
    sum(C[i] * x[i, t] for i in 1:L for t in 1:T) - 0.2 * sum(P_curtail[t] for t in 1:T)
)

# Power balance equaton
for t in 1:T
    @constraint(model, sum(power[i] * x[i, t] for i in 1:L) + P_bc[t] * Bc[t] + P_curtail[t] == P_PV[t] - P_bd[t] * Bd[t] )
end

# SOC estimation
for t in 1:T
    if t == 1
        # Initial SOC, when t is 1
        @constraint(model, SOC[t] == initial_SOC + (ηc * P_bc[t] * Bc[t] + (P_bd[t] * Bd[t])/ ηd)/E_total)
    else
        # consider this equation for SOC update when t > 1
        @constraint(model, SOC[t] == SOC[t - 1] + (ηc * P_bc[t] * Bc[t] + (P_bd[t] * Bd[t])/ ηd)/E_total)
    end
end

# Battery charging limits
@constraint(model, [t in 1:T], Pbc_min <= P_bc[t] <= Pbc_max)

# Battery discharging power limits
@constraint(model, [t in 1:T], Pbd_min >= P_bd[t] >= Pbd_max)

# SOC bounds 
@constraint(model, [t in 1:T], SOC_min <= SOC[t] <= SOC_max)

# Stop simultaneous charging and discharging
for t in 1:T
    @constraint(model, Bc[t] + Bd[t] <= 1)
end

# Final SOC constraint
@constraint(model, SOC[T] >= final_SOC)

# Power Curtailment constraint, need to modify it
@constraint(model, [t in 1:T], P_curtail[t] >= 0)


# Loads Category and Opteration

# 1. SI Devices (Schedulable and Interruptible)
# For all SI devices
si_indices = findall(x -> x == "SI", appliance_names)

# SI devices' constraint
for i in si_indices
    @constraint(model, sum(x[i, t] for t in 1:T) == (1 - skip[i]) * d[i])
    for t in 1:T
        if t < start_time[i] || t >= end_time[i]  # Appliance cannot operate before its allowed time window
            @constraint(model, x[i, t] == 0)  # Cannot operate after allowed time window
        end
    end
end

# 2. SNI devices (schedulable and non-interruptible)
# For all SNI devices
sni_indices = findall(x -> x == "SNI", appliance_names) 

for i in sni_indices
    for t in 1:T
        # Constraint for non-interruptibility (can’t turn off once started)
        @constraint(model, x[i, t] <= 1 - y[i, t])
    end
    for t in 2:T
        # Manage transitions
        @constraint(model, x[i, t - 1] - x[i, t] <= y[i, t])
        # for consistency in transitions
        @constraint(model, y[i, t - 1] <= y[i, t])
    end
    @constraint(model, sum(x[i, t] for t in 1:T) == (1 - skip[i]) * d[i])

    for t in 1:T
        # Appliance cannot operate before its allowed time window
        if t < start_time[i] || t >= end_time[i]
            @constraint(model, x[i, t] == 0)  # Cannot operate after allowed time window
        end
    end
end

# 3. NSNI loads (non-schedulable and non-interruptible)
# For all NSNI devices
nsni_indices = findall(x -> x == "NSNI", appliance_names)

for i in nsni_indices
    for t in 1:T
        # Ensure device operates only for the specified time 
        if t >= start_time[i] && t < start_time[i] + d[i]
            @constraint(model, x[i, t] <= 1 - skip[i])
        else
            # Load must be inactive outside the fixed/specified duration
            @constraint(model, x[i, t] == 0)
        end
    end
    # Run from start to end time
    @constraint(model, sum(x[i, t] for t in start_time[i]:(start_time[i] + d[i] - 1)) == (1 - skip[i]) * d[i])
end

# Solve the optimization problem
optimize!(model)


# Extract and display results
SOC_results = value.(SOC)
P_bc_results = value.(P_bc)
P_bd_results = value.(P_bd)
load_served = [sum(value(x[i, t]) for i in 1:L) for t in times]
P_curtailed = [sum(value(P_curtail[t]) for i in 1:L) for t in times]

# print the output
println("SOC : ", SOC_results)
println("Charging power values (P_bc): ", P_bc_results)
println("Discharging power (P_bd): ", P_bd_results)
println("Load Served: ", load_served)
println("Power Curtailed: ", P_curtailed)
