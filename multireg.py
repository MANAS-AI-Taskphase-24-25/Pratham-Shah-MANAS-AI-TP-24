import math
import numpy as np
import matplotlib.pyplot as plt
import gettingdata

def z_score_normalise(X):
    #Z score normalisation algorithm. Returns z score normalised dataset, means and std. deviations of the features.
    # find the mean of each n columns
    means     = np.mean(X, axis=0)                 # mean will have length n
    # find the standard deviation of each n columns
    stdevs  = np.std(X, axis=0)+0.00001                 # sigma will have length n. +0.00001 to prevent divide by 0 error.
    X_norm = (X - means) / stdevs    

    return (X_norm, means, stdevs)

def get_predictions(testing, weights, bias, correctvals):
  #returns predictions and their corresponding errors.
  #predict output value using test data multiplied by corresponding weights, + bias
  predictions = [np.dot(val, weights) + bias for val in testing]
  i = 0
  errs = []
  r2_numerator = 0
  r2_denominator = 0
  #mean of the correct values, for calculating r2 score.
  correct_avg = sum(correctvals)/len(correctvals)
  for value in predictions:
    #append error in prediction
    errs.append(value - correctvals[i])
    r2_numerator += math.pow(correctvals[i] - value,2) #calculates r2 numerator
    r2_denominator += math.pow(correctvals[i]- correct_avg, 2) #calculates r2 denominator
    i += 1 #looping variable
  mse = sum([err*err for err in errs])/len(errs) #mean squared error
  rmse = math.sqrt(mse) #root mean squared error
  return predictions, mse, rmse, 1 - r2_numerator/r2_denominator

def calculate_cost(X, y, w, b):
    m = X.shape[0] #stores number of training examples (m), to iterate through them
    cost = 0.0
    for i in range(m):                                      #iterates through training examples                            
        value_using_given_params = np.dot(X[i], w) + b      #stores the predicted value of the data, using provided weights and bias
        cost = cost + (value_using_given_params - y[i])**2  #calculates the 'sigma' part of the cost function 
    cost = cost / (2 * m)                                   #1/2m * sigma part = complete cost value          
    return cost

def calculate_gradient(X, y, w, b):
    m,n = X.shape           #(number of examples, number of features)
    dj_dw = np.zeros((n,))
    dj_db = 0.
    for i in range(m):                             
        err = (np.dot(X[i], w) + b) - y[i] #calculates error between actual and predicted value
        for j in range(n):                          
            dj_dw[j] = dj_dw[j] + err * X[i, j]    
        dj_db = dj_db + err                        
    dj_dw = dj_dw / m                                
    dj_db = dj_db / m                                
    return dj_db, dj_dw #returns derivate of cost function wrt bias and weights

def gradient_descent(X, y, w_init, b_init, cost_function, gradient_function, alpha, num_iters):
    # An array to store cost J and w's at each iteration primarily for graphing later
    J_history = []
    w = w_init
    b = b_init
    for i in range(num_iters):
        # Calculate the gradient and update the parameters
        dj_db,dj_dw = gradient_function(X, y, w, b)

        # Update Parameters using w, b, learning rate and gradient
        w = w - alpha * dj_dw            
        b = b - alpha * dj_db          
      
        # Save cost J at each iteration
        J_history.append( cost_function(X, y, w, b))

        if(i%100 == 0):
          predictions, mse, rmse, r2 = get_predictions(X_test, w, b, y_test)
          print(f"At iteration {i} :\nCost Function:  {cost_function(X, y, w, b)}\nMSE: {mse}\nRMSE:  {rmse}\nR2:  {r2}\n")
        
    return w, b, J_history #return final w,b and J history for graphing

def multiple_regression(xvals, yvals, initial_coeffs, initial_affine, iterations, learning_rate):
  w_final, b_final, J_hist = gradient_descent(xvals, yvals, initial_coeffs, initial_affine,
                                                      calculate_cost, calculate_gradient, 
                                                      learning_rate, iterations) #gets the final weights, bias and cost function history
  for index, value in enumerate(w_final):
     print(f"Weight of {list(gettingdata.df)[index]}  :   {value}")
  print(f"Bias value : {b_final}") #prints the weights and bias 
  return w_final, b_final, J_hist

X_train, feature_means, feature_stddevs = z_score_normalise(np.array(gettingdata.data)) #Z-score-normalise the training set, storing the features means and std devs for future reference.
y_train = np.array(gettingdata.prices) #stores training data's corresponding prices

X_test = (z_score_normalise(np.array(gettingdata.tdata))[0] + feature_means)/feature_stddevs #stores z-score-normalised testing data
y_test = np.array(gettingdata.tprices) #stores testing data prices
test_names = gettingdata.test_do["Name"] #stores names of cars in the testing data

b_init = 0
w_init = np.zeros_like(gettingdata.data[0])
fig, (ax1, ax2) = plt.subplots(1,2)

coeffs, affine, hist = multiple_regression(X_train, y_train, np.zeros_like(w_init), 0, 1000, 0.001) #stores final weights, bias and cost function history

predictions, netmse, netrmse, netr2 = get_predictions(X_test, coeffs, affine, y_test) #stores the final predicitons, mean square error, root mse, and r2 score.
for index, prediction in enumerate(predictions):
  print(f"{test_names[index]}:\nGiven Value:  {y_test[index]}\nPredicted Value: {predictions[index]}") #prints predictions

ax2.plot(range(0, len(hist)), hist)     #plots the cost function v/s iterations
ax1.scatter(range(0, len(predictions)),predictions, c='r')  #plots the predicted values in red
ax1.scatter(range(0, len(y_test)),y_test, c='g')        #plots the actual values in green
plt.suptitle("The first graph represents predicted values (red), real values (green)\nThe second graph represents the cost function v/s iterations.")
ax1.grid()
plt.show() #graphs predictions