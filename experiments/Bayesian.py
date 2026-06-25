# Run Bayesian Optimization to do car following calibration iteratively
# Maximize the negative error term
"""
Iteration 0 - Preparation
Run ten simulations(based on demand we generated) and got ten datapoints

Iteration 1-k
1. Random select a parameter set and do simulation
2. Fit the Gaussian process and choose the next point
"""

"""
Challenges:
1. The python library should be installed in the server
2. Run the simulation using python language (easy to solve maybe - refer to xuan's pipeline code)
3. Create space/variable to store the Gaussian Process / data point set
"""
import numpy as np
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import Matern
import sklearn.gaussian_process as gp
from scipy.stats import norm
import pickle


scaling_factor = 1.0
def expected_improvement(x, gaussian_process, evaluated_loss, n_params=1):

    x_to_predict = x.reshape(-1, n_params)
    mu, sigma = gaussian_process.predict(x_to_predict, return_std=True)
    loss_optimum = np.max(evaluated_loss)
    
    # In case sigma equals zero
    with np.errstate(divide='ignore'):
        Z = scaling_factor * (mu - loss_optimum) / sigma
        expected_improvement = (mu - loss_optimum) * norm.cdf(Z) + sigma * norm.pdf(Z)
        expected_improvement[sigma == 0.0] == 0.0

    return expected_improvement

def bayesian_one_step(bounds, x_list, y_list, model, epsilon=1e-7):
    # bounds of the parameters
    n_params = bounds.shape[0]
    # Data structure of x,y -> np.array, column vector
    x = np.array(x_list)
    y = np.array(y_list)


    if model is None: # If iteration=1, do this
        kernel = gp.kernels.Matern() # Use Matern kernal according to literature
        model = GaussianProcessRegressor(kernel=kernel, alpha=1e-5, n_restarts_optimizer=10, normalize_y=True)
        model.fit(x, y)
    else:
        model.fit(x, y)

    x_random = np.random.uniform(bounds[:, 0], bounds[:, 1], size=(10000, n_params))  # Using 10000 random samples
    ei = expected_improvement(x_random, model, y, n_params=n_params)
    next_sample = x_random[np.argmax(ei), :]

    if np.any(np.abs(next_sample - xp) <= epsilon):
        next_sample = np.random.uniform(bounds[:, 0], bounds[:, 1], bounds.shape[0])

    x_list.append(next_sample)

    return x_list, model


"""
Iteration 0 - Run ten simulations with 10 random parameter sets and store the error
"""
# Return a x_list and y_list
# x_list - parameter list(every element is a 4-parameter group):[[1,2,3,4]]
# y_list - rmse?/distribution difference?: [20]



# After iteration 0, we set a criteria for the bounds of parameters
# Will be used in the subsequent BO
bounds = np.array([[1,10],[1,10],[0.1,2],[1.0,5]]) 

# Iteration 1 - Iteration k
# Number of iterations and the initial model
n_iterations = 20
model = None

# File to store the Gaussian process model
gaussian_process_file = 'gaussian_process.pkl'
y_list = []
x_list = []
# Perform the Bayesian optimization loop
for iteration in range(n_iterations):
    print(f"Iteration {iteration + 1}")
    
    # Load the Gaussian process model from the file for iterations greater than 1
    if iteration > 0:
        with open(gaussian_process_file, 'rb') as file:
            model = pickle.load(file)
    
    # Perform one step of Bayesian optimization
    x_list, model = bayesian_one_step(bounds, x_list, y_list, model)
    
    # Store the updated Gaussian process model to the file
    with open(gaussian_process_file, 'wb') as file:
        pickle.dump(model, file)
    
    """
    Do simulation and get the updated y - according to current paramter combination
    """
    y_list.append()


