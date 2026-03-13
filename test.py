import sympy as sp

# 1. Setup symbols
q1, q2 = sp.symbols('q1 q2')
m1, m2, l1, l2, I1, I2 = sp.symbols('m1 m2 l1 l2 I1 I2')
params = (m1, m2, l1, l2, I1, I2) # Define what counts as a "parameter"

# 2. A complex M[1,1] with MULTIPLE functions
# This has a constant part, a cos part, a sin part, and a cos^2 part
M11 = (I1 + m2*l1**2) + (m2*l1*l2)*sp.cos(q2) + (m2*l2**2)*sp.sin(q1) + (m1*l1)*sp.cos(q2)**2
M11 = M11.expand()

print("--- Complex Expression ---")
sp.pprint(M11)

# 3. THE LOOP: Extracting everything automatically
# We use sp.Add.make_args to break the expression at every "+"
results = {}

for term in sp.Add.make_args(M11):
    # For each term (like m2*l1*l2*cos(q2)):
    # .as_independent(*params) splits it into (KinematicPart, ParameterPart)
    # Note: we use *params to tell it ALL these are the constant symbols
    kinematic_part, parameter_part = term.as_independent(*params)
    
    # We group them in a dictionary
    # If the kinematic part is already in there, we add to it (for base parameters)
    if kinematic_part in results:
        results[kinematic_part] += parameter_part
    else:
        results[kinematic_part] = parameter_part

# 4. Show the "Linearization"
print("\n--- Extracted Linear Mapping ---")
for f_q, rho in results.items():
    print(f"Function f(q): {f_q}")
    print(f"Parameter rho: {rho}")
    print("-" * 20)