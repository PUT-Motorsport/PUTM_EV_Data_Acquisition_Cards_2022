/*
 * global_variables.hpp
 *
 *  Created on: May 27, 2022
 *      Author: molso
 */

#ifndef INC_GLOBAL_VARIABLES_HPP_
#define INC_GLOBAL_VARIABLES_HPP_

enum struct Status{
	Power_up,
	Normal_operation,
	Sensor_impossibility,
};

extern Status status;



#endif /* INC_GLOBAL_VARIABLES_HPP_ */
