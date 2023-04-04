//
// Created by itamar on 4/3/23.
//

#ifndef MANIPULATION_PLANNING_UTILS_HPP
#define MANIPULATION_PLANNING_UTILS_HPP

#include <vector>

/// \brief A function that dealing with the discontinuity of the joint angles
/// \param state The state to check
/// \return The state with the joint angles in the range of [-pi, pi]
stateType normalizeAngles(const stateType& state){
    stateType normalized_state = state;
    for (int i = 0; i < state.size(); ++i) {
        if (normalized_state[i] > M_PI){
            normalized_state[i] = normalized_state[i] - 2*M_PI;
        }
        else if (normalized_state[i] < -M_PI){
            normalized_state[i] = normalized_state[i] + 2*M_PI;
        }
    }
    return normalized_state;
}

/// \brief A function that takes the discretization vector and a state, and round it to the nearest discretization value
/// \param state The state to check
/// \param discretization The discretization vector
void roundStateToDiscretization(stateType& state, const stateType& discretization){
    for (int i = 0; i < state.size(); ++i) {
        state[i] = round(state[i]/discretization[i])*discretization[i];
    }
}


#endif //MANIPULATION_PLANNING_UTILS_HPP
