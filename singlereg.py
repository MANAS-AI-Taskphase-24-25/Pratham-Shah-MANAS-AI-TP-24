import numpy as np
import matplotlib.pyplot as plt

fig, (ax1, ax2) = plt.subplots(1,2)

#Function to calculate the cost
def compute_cost(x, y, w, b):
   
    m = x.shape[0] 
    cost = 0
    
    for i in range(m):
        f_wb = w * x[i] + b
        cost = cost + (f_wb - y[i])**2
    total_cost = 1 / (2 * m) * cost

    return total_cost

def compute_gradient(x, y, w, b): 
    """
    Computes the gradient for linear regression 
    Args:
      x (ndarray (m,)): Data, m examples 
      y (ndarray (m,)): target values
      w,b (scalar)    : model parameters  
    Returns
      dj_dw (scalar): The gradient of the cost w.r.t. the parameters w
      dj_db (scalar): The gradient of the cost w.r.t. the parameter b     
     """
    
    # Number of training examples
    m = x.shape[0]    
    dj_dw = 0
    dj_db = 0
    
    for i in range(m):  
        f_wb = w * x[i] + b 
        dj_dw_i = (f_wb - y[i]) * x[i] 
        dj_db_i = f_wb - y[i] 
        dj_db += dj_db_i
        dj_dw += dj_dw_i 
    dj_dw = dj_dw / m 
    dj_db = dj_db / m 
        
    return dj_dw, dj_db

def gradient_descent(x, y, w_in, b_in, alpha, num_iters, cost_function, gradient_function): 
  
    w = w_in # avoid modifying global w_in
    # An array to store cost J and w's at each iteration primarily for graphing later
    J_history = []
    p_history = []
    b = b_in
    w = w_in
    
    for i in range(num_iters):
        # Calculate the gradient and update the parameters using gradient_function
        dj_dw, dj_db = gradient_function(x, y, w , b)     

        # Update Parameters using equation (3) above
        b = b - alpha * dj_db                            
        w = w - alpha * dj_dw                            

        # Save cost J at each iteration
        if i<100000:      # prevent resource exhaustion 
            J_history.append( cost_function(x, y, w , b))
            p_history.append([w,b])

    return w, b, J_history, p_history #return w and J,w history for graphing

def callrec(xs, ys):
    start = 0
    # initialize parameters
    w_init = ys[start]/xs[start]
    b_init = ys[start] - xs[start]
    # some gradient descent settings
    iterations = 10000
    tmp_alpha = 1.0e-2
    # run gradient descent
    w_final, b_final, J_hist, p_hist = gradient_descent(np.asarray(xs) ,np.asarray(ys), w_init, b_init, tmp_alpha, 
    iterations, compute_cost, compute_gradient)

    print(f"(w,b) found by gradient descent: ({w_final:8.4f},{b_final:8.4f})")

    ax1.plot(np.array(range(0, iterations)), J_hist)
    ax1.set_title("Cost Function v/s Iterations")
    ax1.set_ylabel("Cost Function")
    ax1.set_xlabel("Number of Iterations")
    ax2.plot(range(min(xs), max(xs)+ 1), np.asarray([w_final * x + b_final for x in range(min(xs), max(xs) + 1)]))
    ax2.scatter(xs, ys, marker='x', c='r',label='Actual Values')
    ax2.set_title("Model")
    ax2.set_ylabel("Dependenet Variable")
    ax2.set_xlabel("Independent variable")

    fig.suptitle(f"Here is a model for single-variable linear regression.\nThe equation of the line thus formed is:\nY = {w_final:.2f}X + {b_final:.2f}")
    plt.show()

callrec([1, 2, 3, 4, 5], [3, 6.7, 9.8, 12.5, 19])