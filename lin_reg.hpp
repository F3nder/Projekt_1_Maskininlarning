/********************************************************************************
 * @brief Library for implementing linear regression models in C++.
 ********************************************************************************/
#pragma once

#include <vector.hpp>
#include <stdlib.h>
#include <time.h>

namespace yrgo {

/********************************************************************************
 * @brief Class for implementing linear regression models.
 ********************************************************************************/
class LinReg {
  public:

    /********************************************************************************
     * @brief Default constructor, creates empty regression model.
     ********************************************************************************/
    LinReg(void) = default;

    /********************************************************************************
     * @brief Creates new regression model and stores referenced training data.
     * 
     * @param train_in
     *        Reference to vector containing input data (x).
     * @param train_out
     *        Reference to vector containing reference data (y_ref).
     ********************************************************************************/
    LinReg(const container::Vector<double>& train_in, const container::Vector<double>& train_out) {
        LoadTrainingData(train_in, train_out);
    }

    /********************************************************************************
     * @brief Makes a prediction with the specified input value. 
     *        The prediction is calculated as 
     * 
     *                                y_pred = kx + m,
     *  
     * where x is the specified input value, k is the weight and m is the bias.
     *        
     * @param input
     *        The input value (x) to predict with.
     * @return
     *        The predicted value (y_pred).
     ********************************************************************************/
    double Predict(const double input) const { return weight_ * input + bias_; }

    /********************************************************************************
     * @brief Loads training data from referenced vectors.
     * 
     * @param train_in
     *        Reference to vector containing input data (x).
     * @param train_out
     *        Reference to vector containing reference data (y_ref).
     ********************************************************************************/
    void LoadTrainingData(const container::Vector<double>& train_in, 
                          const container::Vector<double>& train_out);

    /********************************************************************************
     * @brief Trains regression model with specified parameters.
     * 
     * @param num_epochs
     *        The number of epochs (turns) to train.
     * @param learning_rate
     *        The learning rate, sets the change rate during errors (default = 0.01).
     ********************************************************************************/
    void Train(const size_t num_epochs, const double learning_rate = 0.01);

  /********************************************************************************
   * @note The private segment is only visible internally (i.e. in this class).
   ********************************************************************************/
  private:
    container::Vector<double> train_in_{};         /* Input values (x). */
    container::Vector<double> train_out_{};        /* Reference values (y_ref). */
    container::Vector<size_t> train_order_{}; /* Stores indexes for training sets. */
    double weight_{};                        /* k-value. */
    double bias_{};                          /* m-value. */

    /********************************************************************************
     * @brief Randomizes the training order before each new epoch. This is done to
     *        prevent that the model is learning due to the order of the training
     *        sets.
     ********************************************************************************/
    void RandomizeTrainingOrder(void);

    /********************************************************************************
     * @brief Optimizes the model by making a prediction and adjusting the 
     *        parameters according to the calculated error.
     * 
     * @param input
     *        The input value of the model (x).
     * @param reference
     *        The reference value of the model (y_ref).
     * @param learning_rate
     *        The learning rate, sets the change rate during errors.
     ********************************************************************************/
    void Optimize(const double input, const double reference, const double learning_rate);

    /********************************************************************************
     * @brief Ensures the the vectors storing the training sets are of equal size. 
     *        If not, the superfluous values of the larger vector is removed.
     ********************************************************************************/
    void MatchTrainingSets(void);

     /********************************************************************************
     * @brief Initializes the training order vector so that it stores the index of
     *        each training set.
     ********************************************************************************/
    void InitTrainOrderVector(void);
 
    /********************************************************************************
     * @brief Initializes the random generator. This function should preferably be
     *        called before using the std::rand function to ensure that the 
     *        randomized sequence is unique each time the program is run.
     ********************************************************************************/
    static void InitRandomGenerator(void);
};

} /* namespace yrgo */